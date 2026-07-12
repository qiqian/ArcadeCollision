using System;

namespace ArcCollision;

/// <summary>Built-in collision category bits.</summary>
public static class CollisionCategories
{
    /// <summary>The category used by collider overloads without an explicit filter.</summary>
    public const uint Default = 1u;

    /// <summary>All 32 collision category bits.</summary>
    public const uint All = uint.MaxValue;
}

/// <summary>
/// Per-collider collision filtering. <see cref="Categories"/> describes what the
/// collider is; <see cref="CollidesWith"/> describes the categories it accepts.
/// Two colliders are eligible only when both filters accept the other collider.
/// </summary>
public readonly struct CollisionFilter : IEquatable<CollisionFilter>
{
    /// <summary>A filter that cannot collide with anything.</summary>
    public static readonly CollisionFilter Disabled = default;

    /// <summary>
    /// Default filter: belongs to <see cref="CollisionCategories.Default"/> and
    /// accepts every category.
    /// </summary>
    public static readonly CollisionFilter Default = new(
        CollisionCategories.Default, CollisionCategories.All);

    public readonly uint Categories;
    public readonly uint CollidesWith;

    public CollisionFilter(uint categories, uint collidesWith = CollisionCategories.All)
    {
        Categories = categories;
        CollidesWith = collidesWith;
    }

    /// <summary>True when this filter has no category or accepted-category bits.</summary>
    public bool IsDisabled => Categories == 0 || CollidesWith == 0;

    /// <summary>Returns true only when both filters accept one another.</summary>
    public bool CanCollideWith(in CollisionFilter other) =>
        (Categories & other.CollidesWith) != 0
        && (other.Categories & CollidesWith) != 0;

    /// <summary>Alias for <see cref="CanCollideWith"/>.</summary>
    public bool Allows(in CollisionFilter other) => CanCollideWith(other);

    public bool Equals(CollisionFilter other) =>
        Categories == other.Categories && CollidesWith == other.CollidesWith;

    public override bool Equals(object? obj) =>
        obj is CollisionFilter other && Equals(other);

    public override int GetHashCode() => DeterministicHash.Combine(
        unchecked((int)Categories), unchecked((int)CollidesWith));

    public static bool operator ==(CollisionFilter left, CollisionFilter right) =>
        left.Equals(right);

    public static bool operator !=(CollisionFilter left, CollisionFilter right) =>
        !left.Equals(right);
}
