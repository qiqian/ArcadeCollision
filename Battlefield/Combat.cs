using System;

namespace ArcCollision.Battlefield;

internal enum Faction { Player, Enemy }

// Values intentionally match CombatSystem.HurtTypes.
internal enum HurtType { Mid = 0, High = 1 }
internal enum Knockback { None = 0, Weak = 60, Medium = 600, Strong = 1200, Massive = 2400 }
internal enum HitShapeKind { Box, HorizontalCapsule }

internal readonly record struct BoxShape(Vec2 Center, Vec2 HalfSize, float Rotation = 0f);
internal readonly record struct ShapeKey(float Time, BoxShape Shape);

/// <summary>
/// One interval in which a Godot CollisionShape2D is enabled. HitId identifies
/// intervals belonging to the same physical hit when an animation changes the
/// shape while it remains enabled.
/// </summary>
internal readonly struct HitWindow
{
    public readonly float Start, End;
    public readonly float Damage;
    public readonly Knockback Knockback;
    public readonly HurtType Hurt;
    public readonly float LaunchAngleDeg;
    public readonly int HitId;
    public readonly HitShapeKind ShapeKind;
    public readonly BoxShape Box;
    public readonly float CapsuleRadius, CapsuleHeight;

    public HitWindow(float start, float end, float damage, Knockback knockback,
        HurtType hurt, float launchAngleDeg, int hitId, BoxShape box)
    {
        Start = start;
        End = end;
        Damage = damage;
        Knockback = knockback;
        Hurt = hurt;
        LaunchAngleDeg = launchAngleDeg;
        HitId = hitId;
        ShapeKind = HitShapeKind.Box;
        Box = box;
        CapsuleRadius = CapsuleHeight = 0f;
    }

    public HitWindow(float start, float end, float damage, Knockback knockback,
        HurtType hurt, float launchAngleDeg, int hitId, Vec2 center,
        float capsuleRadius, float capsuleHeight)
    {
        Start = start;
        End = end;
        Damage = damage;
        Knockback = knockback;
        Hurt = hurt;
        LaunchAngleDeg = launchAngleDeg;
        HitId = hitId;
        ShapeKind = HitShapeKind.HorizontalCapsule;
        Box = new BoxShape(center, Vec2.Zero);
        CapsuleRadius = capsuleRadius;
        CapsuleHeight = capsuleHeight;
    }
}

internal sealed class AttackData
{
    public required string Clip;
    public float Total;
    public float ComboWindow;
    public int ComboNext = -1;
    public HitWindow[] Hits = Array.Empty<HitWindow>();
    public ShapeKey[] HurtShapes = Array.Empty<ShapeKey>();
    public float LungeSpeed, LungeStart, LungeEnd;
    public float SuperArmorStart = float.PositiveInfinity;
    public float SuperArmorEnd = float.NegativeInfinity;
    public float InvulnerableStart = float.PositiveInfinity;
    public float InvulnerableEnd = float.NegativeInfinity;

    public bool TryWindowAt(float time, out HitWindow hit)
    {
        foreach (HitWindow candidate in Hits)
            if (time >= candidate.Start && time < candidate.End)
            {
                hit = candidate;
                return true;
            }
        hit = default;
        return false;
    }

    public BoxShape HurtShapeAt(float time, BoxShape fallback)
    {
        BoxShape result = fallback;
        foreach (ShapeKey key in HurtShapes)
        {
            if (time + 0.0001f < key.Time) break;
            result = key.Shape;
        }
        return result;
    }

    public bool HasSuperArmorAt(float time) => time >= SuperArmorStart && time < SuperArmorEnd;
    public bool IsInvulnerableAt(float time) => time >= InvulnerableStart && time < InvulnerableEnd;

    // Godot's launch_vector is (cos(angle), -sin(angle)); this simulation stores
    // skin height with up-positive, so the equivalent is (cos, +sin).
    public static Vec2 LaunchDir(float angleDeg)
    {
        float angle = angleDeg * MathF.PI / 180f;
        return new Vec2(MathF.Cos(angle), MathF.Sin(angle));
    }
}

internal sealed class CharacterDef
{
    public required string Name;
    public required string SpriteSet;
    public int MaxHealth;
    public float Speed;
    public float Radius;                 // body capsule radius (also used by shadow rendering)
    public float BodyHalfSpine;
    public Vec2 BodyOffset;
    public BoxShape HurtBox;
    public BoxShape HurtMid;
    public BoxShape HurtHigh;
    public float RenderScale;
    public float JumpSpeed;
    public float JumpImpulseTime;
    public float AirControl;
    public float BounceTime;
    public float LandedTime;
    public float GetUpTime;
    public required AttackData[] Combo;
    public AttackData? AirAttack;
    public AttackData? Dash;
    public AttackData? Area;
    public AttackData? Retaliate;

    private static float S(float value) => value * Game.WorldScale;
    private static Vec2 V(float x, float y) => new(S(x), S(y));
    private static BoxShape B(float x, float y, float width, float height, float rotation = 0f) =>
        new(V(x, y), V(width * .5f, height * .5f), rotation);
    private static ShapeKey K(float time, float x, float y, float width, float height, float rotation = 0f) =>
        new(time, B(x, y, width, height, rotation));
    private static HitWindow H(float start, float end, float damage, Knockback kb, HurtType hurt,
        float angle, int id, float x, float y, float width, float height, float rotation = 0f) =>
        new(start, end, damage, kb, hurt, angle, id, B(x, y, width, height, rotation));

    public static readonly CharacterDef Chad = new()
    {
        Name = "CHAD",
        SpriteSet = "chad",
        MaxHealth = 100,
        Speed = S(500),
        Radius = S(20),
        BodyHalfSpine = S((204 - 40) * .5f),
        BodyOffset = V(1, 0),
        HurtBox = B(5, -166.5f, 149, 329),
        HurtMid = B(-70, -156, 149, 332),
        HurtHigh = B(-15, -180.75f, 149, 378.5f),
        RenderScale = Game.WorldScale,
        JumpSpeed = S(1200),
        JumpImpulseTime = 0.0833334f,
        AirControl = .6f,
        BounceTime = 0.0833334f,
        LandedTime = .5f,
        GetUpTime = .166667f,
        Combo = new[]
        {
            new AttackData
            {
                Clip = "attack1", Total = .333334f, ComboWindow = .208333f, ComboNext = 1,
                Hits = new[] { H(.0416667f, .125f, 10, Knockback.Weak, HurtType.High, 15, 0,
                    170, -264, 144, 52) },
                HurtShapes = new[]
                {
                    K(0, 23.4999f, -150.5f, 165, 323),
                    K(.25f, 6, -150.5f, 165, 323),
                },
            },
            new AttackData
            {
                Clip = "attack2", Total = .458333f, ComboWindow = .291667f, ComboNext = 2,
                Hits = new[] { H(.125f, .25f, 15, Knockback.Weak, HurtType.High, 30, 0,
                    231, -261, 222, 73) },
                HurtShapes = new[]
                {
                    K(0, -1, -174.5f, 149, 363),
                    K(.125f, 43, -159.75f, 149, 363),
                    K(.333334f, 5, -168, 149, 363),
                },
            },
            new AttackData
            {
                Clip = "attack3", Total = .708333f,
                Hits = new[] { H(.333333f, .458333f, 30, Knockback.Strong, HurtType.Mid, 30, 0,
                    96.5f, -389.25f, 161, 329.5f) },
                HurtShapes = new[]
                {
                    K(0, -10.75f, -116, 168.5f, 246),
                    K(.208333f, 36, -239, 148, 300),
                    K(.333333f, 57, -259, 148, 340),
                    K(.583333f, 38, -282.5f, 148, 345),
                },
            },
        },
        AirAttack = new AttackData
        {
            Clip = "air_attack", Total = float.PositiveInfinity,
            Hits = new[] { H(.125f, float.PositiveInfinity, 20, Knockback.Strong, HurtType.High, 45, 0,
                175.632f, -225.448f, 261.336f, 79.2264f, .294481f) },
            HurtShapes = new[]
            {
                K(0, 17.5f, -241, 151, 302),
                K(.125f, 35.5f, -279, 163, 292),
            },
        },
    };

    public static readonly CharacterDef Sarge = new()
    {
        Name = "SARGENT",
        SpriteSet = "sarge",
        MaxHealth = 50,
        Speed = S(350),
        Radius = S(18),
        BodyHalfSpine = S((220 - 36) * .5f),
        BodyOffset = Vec2.Zero,
        HurtBox = B(-10, -209, 210, 414),
        HurtMid = B(-40.5f, -199, 221, 414),
        HurtHigh = B(-10, -202, 210, 414),
        RenderScale = Game.WorldScale,
        JumpSpeed = S(1200),
        JumpImpulseTime = .291667f,
        AirControl = .6f,
        BounceTime = .166667f,
        LandedTime = .5f,
        GetUpTime = .583334f,
        Combo = new[]
        {
            new AttackData
            {
                Clip = "attack1", Total = .333334f, ComboWindow = .25f, ComboNext = 1,
                Hits = new[] { H(.208334f, .25f, 3, Knockback.Weak, HurtType.High, 0, 0,
                    192, -338, 208, 63) },
                HurtShapes = new[] { K(0, -10, -209, 210, 414) },
            },
            new AttackData
            {
                Clip = "attack2", Total = .625001f, ComboWindow = .375f, ComboNext = 2,
                Hits = new[] { H(.291667f, .375f, 5, Knockback.Strong, HurtType.Mid, 75, 0,
                    242.689f, -311.254f, 289.027f, 85.6137f, .209387f) },
                HurtShapes = new[] { K(0, -18, -209, 210, 414) },
            },
            new AttackData
            {
                Clip = "attack3", Total = 1.41667f,
                LungeSpeed = S(900), LungeStart = 0, LungeEnd = .25f,
                SuperArmorStart = 0, SuperArmorEnd = 1f,
                Hits = new[]
                {
                    H(0, .166667f, 2, Knockback.Medium, HurtType.High, 90, 0,
                        -5.14774f, -225.311f, 168.383f, 405.783f, .548033f),
                    H(.583334f, .708333f, 8, Knockback.Strong, HurtType.Mid, 5, 1,
                        277.265f, -302.654f, 265.253f, 89.8194f, .20944f),
                },
                HurtShapes = new[]
                {
                    K(0, 38, -219, 246, 434),
                    K(.291667f, 11.5f, -197.5f, 225, 395),
                    K(.583334f, 67.5f, -204, 189, 396),
                    K(1, -25.5f, -194, 165, 392),
                    K(1.08333f, -53, -186, 178, 376),
                    K(1.20833f, -18, -209, 192, 417),
                },
            },
        },
    };

    public static readonly AttackData TaxHurtKnockout = new()
    {
        Clip = "hurt_knockout", Total = .5f,
        SuperArmorStart = 0, SuperArmorEnd = float.PositiveInfinity,
        HurtShapes = new[] { K(0, -40, -239, 200, 444) },
    };

    public static readonly CharacterDef TaxMan = new()
    {
        Name = "TAX MAN",
        SpriteSet = "taxman",
        MaxHealth = 600,
        Speed = S(500),
        Radius = S(18),
        BodyHalfSpine = S((278 - 36) * .5f),
        BodyOffset = V(0, -8),
        // Tax Man's source animations face left; these centers are converted to
        // forward-space so Fighter.Facing can mirror them consistently.
        HurtBox = B(-34.5f, -257, 211, 518),
        HurtMid = B(11, -237, 186, 470),
        HurtHigh = B(-12.5f, -244, 195, 518),
        RenderScale = Game.WorldScale,
        JumpSpeed = S(1200),
        JumpImpulseTime = 0,
        AirControl = .6f,
        BounceTime = 0,
        LandedTime = 0,
        GetUpTime = 0,
        Combo = new[]
        {
            new AttackData
            {
                Clip = "attack_combo", Total = 1.66667f,
                LungeSpeed = S(600), LungeStart = .708333f, LungeEnd = 1.16667f,
                SuperArmorStart = 0, SuperArmorEnd = 1.66667f,
                Hits = new[]
                {
                    H(.0833334f, .166667f, 5, Knockback.Weak, HurtType.High, 10, 0,
                        254, -324.75f, 228, 159.5f),
                    H(.166667f, .25f, 5, Knockback.Weak, HurtType.High, 10, 0,
                        268, -365, 200, 148),
                    H(.416667f, .5f, 3, Knockback.Strong, HurtType.High, 30, 1,
                        256, -392, 292, 138),
                    H(1.08333f, 1.16667f, 20, Knockback.Strong, HurtType.Mid, 330, 2,
                        196, -404, 292, 114, .425743f),
                    H(1.16667f, 1.25f, 20, Knockback.Strong, HurtType.Mid, 330, 2,
                        326.342f, -298.513f, 324.188f, 114, .557359f),
                },
                HurtShapes = new[]
                {
                    K(0, -323.5f, -253, 353, 496),
                    K(.0833334f, 78, -247, 362, 496),
                    K(.166667f, 165.5f, -247, 283, 496),
                    K(.375f, 26, -247, 264, 496),
                    K(.666667f, -93, -247, 248, 496),
                    K(.708334f, 48, -247, 250, 496),
                    K(.791667f, 5, -247, 228, 496),
                    K(.833334f, -34.323f, -283.347f, 193.394f, 547, -.333358f),
                    K(1.08333f, 11.5f, -283.347f, 223, 547),
                    K(1.16667f, 60.5f, -244.423f, 321, 469.153f),
                    K(1.375f, 32.5f, -244.423f, 219, 469.153f),
                    K(1.45833f, -37.5f, -239.5f, 213, 479),
                },
            },
        },
        Dash = new AttackData
        {
            Clip = "attack_dash", Total = 1.5f,
            SuperArmorStart = 0, SuperArmorEnd = 1.5f,
            Hits = new[]
            {
                H(.916666f, 1.166666f, 15, Knockback.Massive, HurtType.Mid, 45, 0,
                    216, -433, 440, 110, .550344f),
            },
            HurtShapes = new[] { K(.833333f, 0, -258, 252, 518) },
        },
        Area = new AttackData
        {
            Clip = "attack_area", Total = 4.20834f,
            SuperArmorStart = 0, SuperArmorEnd = 4.20834f,
            Hits = new[]
            {
                H(2f, 2.04167f, 5, Knockback.None, HurtType.High, 0, 0, 99, -445, 496, 1012.5f),
                H(2.08334f, 2.16667f, 5, Knockback.None, HurtType.High, 0, 1, 99, -445, 496, 1012.5f),
                H(2.20834f, 2.25f, 5, Knockback.None, HurtType.High, 0, 2, 115.5f, -445, 665, 1012.5f),
                H(2.29167f, 2.33334f, 5, Knockback.None, HurtType.High, 0, 3, 29.5f, -445, 913, 1012.5f),
                H(2.375f, 2.41667f, 5, Knockback.None, HurtType.High, 0, 4, 51, -445, 622, 1012.5f),
                H(2.45834f, 2.5f, 5, Knockback.None, HurtType.High, 0, 5, 51, -445, 622, 1012.5f),
                H(2.54167f, 2.58334f, 5, Knockback.None, HurtType.High, 0, 6, 81.5f, -445, 683, 1012.5f),
                new HitWindow(2.66667f, 3.33333f, 15, Knockback.Massive, HurtType.Mid, 45, 7,
                    Vec2.Zero, S(210), S(1360)),
            },
            HurtShapes = new[]
            {
                K(0, -22, -262.5f, 220, 541),
                K(.125f, 5.5f, -215.5f, 263, 447),
                K(1.625f, 34.5f, -231.5f, 213, 479),
                K(1.79167f, 28.5f, -231.5f, 191, 479),
                K(2.125f, -12, -260, 174, 528),
                K(2.41667f, 47, -391, 202, 454),
                K(2.5f, -7, -198, 240, 386),
                K(2.54167f, 38, -123.5f, 202, 377),
                K(3.33333f, -45.5f, -249, 229, 496),
            },
        },
        Retaliate = new AttackData
        {
            Clip = "retaliate", Total = .5f,
            SuperArmorStart = 0, SuperArmorEnd = .5f,
            InvulnerableStart = 0, InvulnerableEnd = .5f,
            Hits = new[]
            {
                H(.166667f, .333333f, 5, Knockback.Strong, HurtType.High, 45, 0,
                    60, -410.5f, 840, 331),
            },
            HurtShapes = new[]
            {
                K(0, -50, -220, 170, 460),
                K(.166667f, 40, -227, 204, 460),
                K(.25f, 49, -280, 216, 528),
            },
        },
    };
}
