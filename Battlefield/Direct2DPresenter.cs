using System;
using System.Drawing;
using System.Drawing.Imaging;
using Vortice.Direct2D1;
using Vortice.Mathematics;
using DxgiFormat = Vortice.DXGI.Format;
using D2DPixelFormat = Vortice.DCommon.PixelFormat;
using D2DAlphaMode = Vortice.DCommon.AlphaMode;
using DrawingSize = System.Drawing.Size;
using static Vortice.Direct2D1.D2D1;

namespace ArcCollision.Battlefield;

/// <summary>
/// Hardware presentation path. Direct2D's HWND render target is backed by
/// Direct3D; the CPU canvas is uploaded once per completed frame and scaled by
/// the GPU instead of GDI+ stretching it in WM_PAINT.
/// </summary>
internal sealed class Direct2DPresenter : IDisposable
{
    private readonly ID2D1Factory _factory;
    private ID2D1HwndRenderTarget _target;
    private ID2D1Bitmap? _frame;
    private DrawingSize _frameSize;
    private DrawingSize _targetSize;

    public Direct2DPresenter(IntPtr hwnd, DrawingSize targetSize)
    {
        _factory = D2D1CreateFactory<ID2D1Factory>(FactoryType.SingleThreaded);
        _targetSize = NonEmpty(targetSize);
        _target = CreateTarget(hwnd, _targetSize);
    }

    public void Resize(DrawingSize size)
    {
        size = NonEmpty(size);
        if (size == _targetSize) return;
        _target.Resize(new SizeI(size.Width, size.Height));
        _targetSize = size;
    }

    public void Present(Bitmap source, DrawingSize destinationSize)
    {
        Resize(destinationSize);
        EnsureFrame(source.Size);

        Rectangle bounds = new(0, 0, source.Width, source.Height);
        BitmapData data = source.LockBits(bounds, ImageLockMode.ReadOnly,
            System.Drawing.Imaging.PixelFormat.Format32bppPArgb);
        try
        {
            _frame!.CopyFromMemory(data.Scan0, (uint)data.Stride);
        }
        finally
        {
            source.UnlockBits(data);
        }

        _target.BeginDraw();
        _target.Transform = System.Numerics.Matrix3x2.Identity;
        _target.Clear(new Vortice.Mathematics.Color4(0.047f, 0.047f, 0.063f, 1f));
        _target.DrawBitmap(_frame!,
            new Rect(0, 0, _targetSize.Width, _targetSize.Height),
            1f, BitmapInterpolationMode.Linear,
            new Rect(0, 0, source.Width, source.Height));
        _target.EndDraw();
    }

    private ID2D1HwndRenderTarget CreateTarget(IntPtr hwnd, DrawingSize size)
    {
        HwndRenderTargetProperties hwndProperties = new()
        {
            Hwnd = hwnd,
            PixelSize = new SizeI(size.Width, size.Height),
            PresentOptions = PresentOptions.Immediately,
        };
        return _factory.CreateHwndRenderTarget(new RenderTargetProperties(), hwndProperties);
    }

    private void EnsureFrame(DrawingSize size)
    {
        if (_frame != null && _frameSize == size) return;
        _frame?.Dispose();
        BitmapProperties properties = new(new D2DPixelFormat(
            DxgiFormat.B8G8R8A8_UNorm, D2DAlphaMode.Premultiplied), 96f, 96f);
        _frame = _target.CreateBitmap(new SizeI(size.Width, size.Height),
            IntPtr.Zero, 0, properties);
        _frameSize = size;
    }

    private static DrawingSize NonEmpty(DrawingSize size) =>
        new(Math.Max(1, size.Width), Math.Max(1, size.Height));

    public void Dispose()
    {
        _frame?.Dispose();
        _target.Dispose();
        _factory.Dispose();
    }
}
