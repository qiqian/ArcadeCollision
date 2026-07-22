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

        // netstandard2.1 has no NativeMemory; use the HGlobal allocator. Realloc
        // requires a non-null block, so the first growth (handle == 0) allocates.
        // Both throw OutOfMemoryException on failure, matching the old null check.
        IntPtr byteCount = (IntPtr)checked((long)newCapacity * sizeof(T));
        IntPtr resized = handle == IntPtr.Zero
            ? Marshal.AllocHGlobal(byteCount)
            : Marshal.ReAllocHGlobal(handle, byteCount);
        SetHandle(resized);
        Capacity = newCapacity;
        return (T*)resized;
    }

    protected override bool ReleaseHandle()
    {
        Marshal.FreeHGlobal(handle);
        handle = IntPtr.Zero;
        Capacity = 0;
        return true;
    }
}
