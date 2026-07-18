using Microsoft.Win32.SafeHandles;
using System.Runtime.InteropServices;

namespace ArcCollision.Wrapper;

/// <summary>
/// Reusable unmanaged storage for blittable P/Invoke scratch data. SafeHandle
/// gives the buffer a finalizer fallback while owners still dispose it eagerly.
/// The owning wrapper objects are single-threaded, so resizing never races a call.
/// </summary>
internal sealed unsafe class NativeBuffer<T> : SafeHandleZeroOrMinusOneIsInvalid
    where T : unmanaged
{
    public int Capacity { get; private set; }

    public NativeBuffer() : base(true) { }

    public T* EnsureCapacity(int required)
    {
        if (required < 0) throw new ArgumentOutOfRangeException(nameof(required));
        if (IsClosed) throw new ObjectDisposedException(nameof(NativeBuffer<T>));
        if (required == 0) return null;
        if (required <= Capacity) return (T*)handle;

        int newCapacity = Capacity == 0 ? 16 : Capacity;
        while (newCapacity < required)
        {
            if (newCapacity > int.MaxValue / 2)
            {
                newCapacity = required;
                break;
            }
            newCapacity *= 2;
        }

        nuint byteCount = checked((nuint)newCapacity * (nuint)sizeof(T));
        void* resized = NativeMemory.Realloc((void*)handle, byteCount);
        if (resized == null) throw new OutOfMemoryException();
        SetHandle((IntPtr)resized);
        Capacity = newCapacity;
        return (T*)resized;
    }

    protected override bool ReleaseHandle()
    {
        NativeMemory.Free((void*)handle);
        handle = IntPtr.Zero;
        Capacity = 0;
        return true;
    }
}
