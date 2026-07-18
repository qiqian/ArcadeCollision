namespace ArcCollision.Ref;

/// <summary>Public limits of the deterministic 24.8 fixed-point core.</summary>
public static class CollisionLimits
{
    public const float GridSize = 1f / 256f;
    public const float MaxCoordinate = 500_000_000f / 256f;
}

/// <summary>Initial capacity and broadphase settings for an <see cref="ArcWorld"/>.</summary>
public readonly struct ArcWorldOptions
{
    public readonly float FatMargin;
    public readonly int InitialColliderCapacity;
    public readonly int InitialPairCapacity;

    public ArcWorldOptions()
        : this(16f, 16, 16)
    {
    }

    public ArcWorldOptions(
        float fatMargin = 16f,
        int initialColliderCapacity = 16,
        int initialPairCapacity = 16)
    {
        if (!float.IsFinite(fatMargin) || fatMargin < 0f)
            throw new ArgumentOutOfRangeException(nameof(fatMargin));
        if (initialColliderCapacity is < 0 or > ArcWorld.MaxColliderCount)
            throw new ArgumentOutOfRangeException(nameof(initialColliderCapacity));
        if (initialPairCapacity < 0)
            throw new ArgumentOutOfRangeException(nameof(initialPairCapacity));
        FatMargin = fatMargin;
        InitialColliderCapacity = initialColliderCapacity;
        InitialPairCapacity = initialPairCapacity;
    }
}
