using System;

namespace ArcCollision.Battlefield;

/// <summary>
/// Immutable poison-pool geometry shared by the native collision world and the
/// renderer. Keeping the original vertices makes the visible pool and its
/// trigger use exactly the same polygon.
/// </summary>
internal sealed class PoisonPool
{
    public Vec2[] Vertices { get; }
    public Polygon Geometry { get; }
    public Shape Collider { get; }
    public Aabb Bounds { get; }
    public Vec2 Center => Bounds.Center;
    public float VisualPhase { get; }

    public PoisonPool(ReadOnlySpan<Vec2> vertices, float visualPhase)
    {
        if (vertices.Length < 3)
            throw new ArgumentException("A poison pool needs at least three vertices.", nameof(vertices));

        Vertices = vertices.ToArray();
        Geometry = new Polygon(Vertices);
        Collider = new Shape(Geometry);
        Bounds = Geometry.Bounds;
        VisualPhase = visualPhase;
    }
}
