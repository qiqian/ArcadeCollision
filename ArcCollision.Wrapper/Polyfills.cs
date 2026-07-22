using System;
using System.Runtime.CompilerServices;

// Shims for BCL APIs that ArcCollision uses but netstandard2.1 (the Unity 2022
// target) does not provide. Each is internal and behaviourally identical to the
// framework method it stands in for.
namespace ArcCollision.Wrapper
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
    }

    internal static class Bits
    {
        /// <summary>
        /// <c>BitConverter.SingleToUInt32Bits</c> (.NET 6+) as the unsigned view of
        /// the netstandard2.1 <c>SingleToInt32Bits</c>; identical bit pattern.
        /// </summary>
        public static uint SingleToUInt32Bits(float value) =>
            unchecked((uint)BitConverter.SingleToInt32Bits(value));
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
