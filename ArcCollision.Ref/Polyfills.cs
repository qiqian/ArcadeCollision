using System;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.CompilerServices;

// Shims for BCL APIs that ArcCollision uses but netstandard2.1 (the Unity 2022
// target) does not provide. Each is internal and behaviourally identical to the
// framework method it stands in for, so the same source compiles unchanged on a
// newer runtime were the target ever raised.
namespace ArcCollision.Ref
{
    internal static class Throw
    {
        /// <summary>netstandard2.1 lacks <c>ArgumentNullException.ThrowIfNull</c>.</summary>
        public static void IfNull(
            object? argument,
            [CallerArgumentExpression("argument")] string? paramName = null)
        {
            if (argument is null) throw new ArgumentNullException(paramName);
        }

        /// <summary>Stands in for <c>ObjectDisposedException.ThrowIf</c> (.NET 7+).</summary>
        public static void IfDisposed(bool disposed, object instance)
        {
            if (disposed) throw new ObjectDisposedException(instance?.GetType().FullName);
        }
    }

    internal static class Bits
    {
        /// <summary>
        /// <c>System.Numerics.BitOperations.Log2</c> is absent from netstandard2.1.
        /// Floor(log2(value)) = index of the highest set bit; 0 when value is 0,
        /// matching the framework method the call sites relied on.
        /// </summary>
        public static int Log2(ulong value)
        {
            int result = 0;
            while ((value >>= 1) != 0) result++;
            return result;
        }

        /// <summary>
        /// <c>BitConverter.SingleToUInt32Bits</c> (.NET 6+) as the unsigned view of
        /// the netstandard2.1 <c>SingleToInt32Bits</c>; identical bit pattern.
        /// </summary>
        public static uint SingleToUInt32Bits(float value) =>
            unchecked((uint)BitConverter.SingleToInt32Bits(value));
    }

    /// <summary>
    /// Stands in for <c>CollectionsMarshal.AsSpan</c>. <see cref="List{T}"/> stores
    /// its elements in a private <c>_items</c> array on every runtime ArcCollision
    /// targets (CoreCLR, Mono/Unity, IL2CPP); fetching that reference-typed field
    /// returns the array itself (a reference conversion, no boxing), so a span over
    /// the first <see cref="List{T}.Count"/> items stays allocation-free on the hot
    /// query-sort paths. The field handle is cached once per element type.
    /// </summary>
    internal static class ListMarshal
    {
        public static Span<T> AsSpan<T>(List<T> list) =>
            new Span<T>((T[])ListItemsField<T>.Value.GetValue(list)!, 0, list.Count);

        /// <summary>
        /// <see cref="List{T}.EnsureCapacity"/> is .NET 6+; grow via the
        /// long-standing <see cref="List{T}.Capacity"/> setter, never shrinking.
        /// </summary>
        public static void EnsureCapacity<T>(List<T> list, int capacity)
        {
            if (list.Capacity < capacity) list.Capacity = capacity;
        }

        private static class ListItemsField<T>
        {
            internal static readonly FieldInfo Value =
                typeof(List<T>).GetField(
                    "_items", BindingFlags.Instance | BindingFlags.NonPublic)
                ?? throw new PlatformNotSupportedException(
                    "List<T> backing field '_items' is unavailable on this runtime; "
                    + "ArcCollision.Ref requires it for in-place sorting.");
        }
    }
}

namespace System.Runtime.CompilerServices
{
    // Recognised by the C# compiler purely by full name, so defining it here
    // lets Throw.IfNull capture the argument expression on netstandard2.1.
    [AttributeUsage(AttributeTargets.Parameter, AllowMultiple = false, Inherited = false)]
    internal sealed class CallerArgumentExpressionAttribute : Attribute
    {
        public CallerArgumentExpressionAttribute(string parameterName) =>
            ParameterName = parameterName;

        public string ParameterName { get; }
    }
}
