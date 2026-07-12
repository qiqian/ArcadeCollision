namespace ArcCollision;

/// <summary>
/// Result of a discrete (static) overlap test.
///
/// When <see cref="Colliding"/> is true, <see cref="Normal"/> points from shape
/// A towards shape B and <see cref="Depth"/> is the penetration distance along
/// that normal. An exact touch reports <c>Colliding</c> with <c>Depth == 0</c>.
///
/// <para><b>Separation contract.</b> <see cref="SeparationForA"/> /
/// <see cref="SeparationForB"/> separate the complete operands in one step after
/// fixed-grid quantization. Convex pairs return their MTV. Concave unions may
/// return a longer accumulated or bounds-based separation when a single convex
/// sub-piece MTV would enter another piece.</para>
///
/// <para><b>Contact accuracy.</b> <see cref="Contact"/> is a stable contact hint.
/// During penetration it may be the midpoint between opposing witness surfaces,
/// rather than a point on either original surface. SAT contacts are additionally
/// clamped into the operands' overlapping world bounds. Compute a clipped contact
/// manifold externally if exact surface anchors are required.</para>
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

    /// <summary>
    /// Translation that pushes A out of B. It is the MTV for convex pairs;
    /// concave unions may return a slightly longer verified separation.
    /// </summary>
    public Vec2 SeparationForA => Normal * -Depth;

    /// <summary>
    /// Translation that pushes B out of A. It is the MTV for convex pairs;
    /// concave unions may return a slightly longer verified separation.
    /// </summary>
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
