namespace ArcCollision;

/// <summary>
/// Result of a discrete (static) overlap test.
///
/// When <see cref="Colliding"/> is true, <see cref="Normal"/> points from shape
/// A towards shape B and <see cref="Depth"/> is the penetration distance. Moving
/// A by <c>-Normal * Depth</c> (or B by <c>+Normal * Depth</c>) separates them.
/// </summary>
public readonly struct Manifold
{
    public readonly bool Colliding;
    public readonly Vec2 Normal;
    public readonly float Depth;
    public readonly Vec2 Contact;

    public Manifold(bool colliding, Vec2 normal, float depth, Vec2 contact)
    {
        Colliding = colliding;
        Normal = normal;
        Depth = depth;
        Contact = contact;
    }

    public static readonly Manifold None = new(false, Vec2.Zero, 0f, Vec2.Zero);

    /// <summary>The minimum translation vector to push A out of B.</summary>
    public Vec2 SeparationForA => Normal * -Depth;

    /// <summary>The minimum translation vector to push B out of A.</summary>
    public Vec2 SeparationForB => Normal * Depth;
}

/// <summary>
/// Result of a swept (continuous) test between a moving shape and a static one.
///
/// <see cref="Hit"/> is true when a collision happens within the motion.
/// <see cref="Time"/> is the fraction of the motion (0..1) at first contact,
/// <see cref="Normal"/> is the surface normal at the contact point and
/// <see cref="Point"/> is the world-space contact position.
/// </summary>
public readonly struct SweepHit
{
    public readonly bool Hit;
    public readonly float Time;
    public readonly Vec2 Normal;
    public readonly Vec2 Point;

    public SweepHit(bool hit, float time, Vec2 normal, Vec2 point)
    {
        Hit = hit;
        Time = time;
        Normal = normal;
        Point = point;
    }

    public static readonly SweepHit Miss = new(false, 1f, Vec2.Zero, Vec2.Zero);
}
