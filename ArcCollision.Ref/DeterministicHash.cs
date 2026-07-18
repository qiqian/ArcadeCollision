namespace ArcCollision.Ref;

internal static class DeterministicHash
{
    private const uint Offset = 2166136261u;
    private const uint Prime = 16777619u;

    public static int Combine(int a, int b)
    {
        uint hash = Add(Add(Offset, unchecked((uint)a)), unchecked((uint)b));
        return unchecked((int)hash);
    }

    public static int Combine(int a, uint b, uint c)
    {
        uint hash = Add(Offset, unchecked((uint)a));
        hash = Add(hash, b);
        hash = Add(hash, c);
        return unchecked((int)hash);
    }

    public static int Float(float value)
    {
        uint bits = BitConverter.SingleToUInt32Bits(value);
        if ((bits & 0x7FFFFFFFu) == 0) return 0;
        if ((bits & 0x7F800000u) == 0x7F800000u
            && (bits & 0x007FFFFFu) != 0)
            bits = 0x7FC00000u;
        return unchecked((int)bits);
    }

    private static uint Add(uint hash, uint value) =>
        unchecked((hash ^ value) * Prime);
}
