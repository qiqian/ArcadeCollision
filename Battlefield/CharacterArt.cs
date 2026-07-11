using System;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace ArcCollision.Battlefield;

/// <summary>
/// Draws a <see cref="Fighter"/> as an articulated, side-view arcade character
/// (Three-Kingdoms flavour) directly with GDI+ — no sprite assets required.
/// Poses (walk / attack / hurt / death) are derived from the fighter's state so
/// every hit and swing reads clearly.
/// </summary>
internal static class CharacterArt
{
    private struct Palette
    {
        public Color Skin, Armor, Trim, Cloth, Metal, Slash;
    }

    private static Palette PaletteFor(Fighter f)
    {
        if (f.Faction == Faction.Player)
            return new Palette
            {
                Skin = Color.FromArgb(238, 205, 172),
                Armor = Color.FromArgb(64, 120, 226),
                Trim = Color.FromArgb(244, 205, 96),
                Cloth = Color.FromArgb(206, 54, 62),
                Metal = Color.FromArgb(224, 230, 240),
                Slash = Color.FromArgb(150, 230, 255),
            };
        if (f.Kind == EnemyKind.Brute)
            return new Palette
            {
                Skin = Color.FromArgb(150, 170, 140),
                Armor = Color.FromArgb(74, 92, 74),
                Trim = Color.FromArgb(40, 54, 44),
                Cloth = Color.FromArgb(90, 70, 40),
                Metal = Color.FromArgb(180, 190, 195),
                Slash = Color.FromArgb(255, 150, 80),
            };
        return new Palette
        {
            Skin = Color.FromArgb(212, 180, 150),
            Armor = Color.FromArgb(158, 62, 56),
            Trim = Color.FromArgb(96, 40, 36),
            Cloth = Color.FromArgb(70, 60, 55),
            Metal = Color.FromArgb(210, 214, 220),
            Slash = Color.FromArgb(255, 240, 210),
        };
    }

    public static void DrawShadow(Graphics g, Fighter f)
    {
        float a = f.Dead ? Math.Clamp(f.DeathTimer / 0.5f, 0f, 1f) : 1f;
        float rx = f.Radius * 1.15f * f.Scale;
        float ry = rx * 0.42f;
        using var b = new SolidBrush(Color.FromArgb((int)(80 * a), 0, 0, 0));
        g.FillEllipse(b, f.Pos.X - rx, f.Pos.Y - ry * 0.5f, rx * 2, ry);
    }

    public static void Draw(Graphics g, Fighter f)
    {
        Palette pal = PaletteFor(f);

        float alpha = 1f;
        if (f.Dead)
            alpha = Math.Clamp(f.DeathTimer / 0.45f, 0f, 1f);
        if (!f.Dead && f.Invuln > 0f && f.Faction == Faction.Player)
            alpha *= MathF.Sin(f.AnimClock * 42f) > 0 ? 1f : 0.35f;

        GraphicsState gs = g.Save();
        g.TranslateTransform(f.Pos.X, f.Pos.Y);
        if (f.Dead)
        {
            float topple = (1f - Math.Clamp(f.DeathTimer / 1.1f, 0f, 1f)) * 82f;
            g.RotateTransform((f.KnockVel.X >= 0 ? 1f : -1f) * topple);
        }
        g.ScaleTransform(f.Facing * f.Scale, f.Scale);

        // hurt flash tints the armour toward white
        float flash = Math.Clamp(f.HurtFlash / 0.14f, 0f, 1f);
        Color armor = Blend(pal.Armor, Color.White, flash * 0.7f);
        Color skin = Blend(pal.Skin, Color.White, flash * 0.7f);

        // ---- pose parameters ----
        float bob = f.State == AnimState.Walk
            ? MathF.Abs(MathF.Sin(f.WalkPhase)) * 3f
            : MathF.Sin(f.AnimClock * 2.5f) * 1.2f;
        float legSwing = f.State == AnimState.Walk ? MathF.Sin(f.WalkPhase) * 11f : 3f;
        float leanX = f.Lean * (f.State == AnimState.Attack ? 7f : -5f);
        if (f.State == AnimState.Hurt)
            leanX += MathF.Sin(f.StateTime * 70f) * 2f;

        const float hipY = -30f, shoulderY = -52f, headY = -63f;
        var hip = new PointF(leanX * 0.4f, hipY - bob);
        var shoulder = new PointF(leanX, shoulderY - bob);
        var head = new PointF(leanX * 1.2f, headY - bob);

        // ---- cape (player) ----
        if (f.Faction == Faction.Player)
        {
            float sway = MathF.Sin(f.AnimClock * 3f + 1f) * 3f;
            using var cape = new SolidBrush(A(pal.Cloth, alpha * 0.95f));
            var path = new GraphicsPath();
            path.AddPolygon(new[]
            {
                new PointF(shoulder.X - 8, shoulder.Y),
                new PointF(shoulder.X + 4, shoulder.Y - 2),
                new PointF(hip.X - 6 - sway, hip.Y + 2),
                new PointF(hip.X - 16 - sway, 2),
                new PointF(hip.X - 22 - sway * 1.5f, 6),
            });
            g.FillPath(cape, path);
            path.Dispose();
        }

        // ---- legs ----
        using (var leg = new Pen(A(pal.Trim, alpha), 7f * 1f) { StartCap = LineCap.Round, EndCap = LineCap.Round })
        {
            g.DrawLine(leg, hip.X, hip.Y, -legSwing, 0);
            g.DrawLine(leg, hip.X, hip.Y, legSwing, 0);
        }
        using (var boot = new SolidBrush(A(Blend(pal.Trim, Color.Black, 0.3f), alpha)))
        {
            g.FillEllipse(boot, -legSwing - 4, -3, 9, 6);
            g.FillEllipse(boot, legSwing - 4, -3, 9, 6);
        }

        // ---- back arm ----
        using (var arm = new Pen(A(Blend(armor, Color.Black, 0.15f), alpha), 5f) { StartCap = LineCap.Round, EndCap = LineCap.Round })
            g.DrawLine(arm, shoulder.X, shoulder.Y, shoulder.X - 8, shoulder.Y + 10);

        // ---- torso ----
        using (var torso = new SolidBrush(A(armor, alpha)))
        {
            var path = new GraphicsPath();
            path.AddPolygon(new[]
            {
                new PointF(hip.X - 9, hip.Y + 2),
                new PointF(shoulder.X - 11, shoulder.Y),
                new PointF(shoulder.X + 11, shoulder.Y),
                new PointF(hip.X + 9, hip.Y + 2),
            });
            g.FillPath(torso, path);
            path.Dispose();
        }
        using (var belt = new Pen(A(pal.Trim, alpha), 3f))
            g.DrawLine(belt, hip.X - 9, hip.Y + 1, hip.X + 9, hip.Y + 1);
        using (var chest = new Pen(A(pal.Trim, alpha), 2f))
            g.DrawLine(chest, shoulder.X - 6, shoulder.Y + 4, hip.X + 3, hip.Y - 2);

        // ---- head ----
        using (var neck = new Pen(A(skin, alpha), 5f) { StartCap = LineCap.Round, EndCap = LineCap.Round })
            g.DrawLine(neck, shoulder.X, shoulder.Y, head.X, head.Y + 5);
        using (var hd = new SolidBrush(A(skin, alpha)))
            g.FillEllipse(hd, head.X - 7, head.Y - 8, 14, 16);
        DrawHelmet(g, f, pal, head, alpha);

        // ---- front arm + weapon + slash ----
        DrawWeapon(g, f, pal, shoulder, alpha);

        g.Restore(gs);
    }

    private static void DrawHelmet(Graphics g, Fighter f, Palette pal, PointF head, float alpha)
    {
        using var metal = new SolidBrush(A(pal.Armor, alpha));
        using var trim = new SolidBrush(A(pal.Trim, alpha));

        if (f.Faction == Faction.Player)
        {
            // rounded helm + red plume
            var helm = new GraphicsPath();
            helm.AddPolygon(new[]
            {
                new PointF(head.X - 8, head.Y - 2),
                new PointF(head.X - 6, head.Y - 10),
                new PointF(head.X + 6, head.Y - 10),
                new PointF(head.X + 8, head.Y - 2),
            });
            g.FillPath(metal, helm);
            helm.Dispose();
            using (var rim = new Pen(A(pal.Trim, alpha), 2f))
                g.DrawLine(rim, head.X - 8, head.Y - 2, head.X + 8, head.Y - 2);

            float sway = MathF.Sin(f.AnimClock * 4f) * 2f;
            using var plume = new SolidBrush(A(pal.Cloth, alpha));
            var p = new GraphicsPath();
            p.AddPolygon(new[]
            {
                new PointF(head.X, head.Y - 9),
                new PointF(head.X - 10 - sway, head.Y - 20),
                new PointF(head.X - 4 - sway, head.Y - 20),
                new PointF(head.X + 2, head.Y - 10),
            });
            g.FillPath(plume, p);
            p.Dispose();
        }
        else if (f.Kind == EnemyKind.Brute)
        {
            var helm = new GraphicsPath();
            helm.AddPolygon(new[]
            {
                new PointF(head.X - 8, head.Y - 1),
                new PointF(head.X - 7, head.Y - 9),
                new PointF(head.X + 7, head.Y - 9),
                new PointF(head.X + 8, head.Y - 1),
            });
            g.FillPath(metal, helm);
            helm.Dispose();
            using var horn = new SolidBrush(A(pal.Metal, alpha));
            var h1 = new GraphicsPath();
            h1.AddPolygon(new[] { new PointF(head.X - 7, head.Y - 6), new PointF(head.X - 14, head.Y - 12), new PointF(head.X - 6, head.Y - 9) });
            var h2 = new GraphicsPath();
            h2.AddPolygon(new[] { new PointF(head.X + 7, head.Y - 6), new PointF(head.X + 14, head.Y - 12), new PointF(head.X + 6, head.Y - 9) });
            g.FillPath(horn, h1);
            g.FillPath(horn, h2);
            h1.Dispose();
            h2.Dispose();
        }
        else
        {
            var cap = new GraphicsPath();
            cap.AddPolygon(new[]
            {
                new PointF(head.X - 8, head.Y - 1),
                new PointF(head.X - 5, head.Y - 9),
                new PointF(head.X + 5, head.Y - 9),
                new PointF(head.X + 8, head.Y - 1),
            });
            g.FillPath(trim, cap);
            cap.Dispose();
        }
    }

    private static void DrawWeapon(Graphics g, Fighter f, Palette pal, PointF shoulder, float alpha)
    {
        var hand = new PointF(shoulder.X + 6, shoulder.Y + 3);
        float deg = WeaponAngle(f);
        float rad = deg * MathF.PI / 180f;
        var dir = new PointF(MathF.Cos(rad), MathF.Sin(rad));

        // slash arc during the active strike window
        if (f.AttackActive)
        {
            float u = (f.AttackElapsed - f.AttackWindup) / MathF.Max(0.01f, f.AttackActiveDur);
            float sa = 1f - Math.Clamp(u, 0f, 1f);
            float len = f.Reach + f.Radius * 0.4f;
            using var wide = new Pen(A(pal.Slash, alpha * sa * 0.55f), 10f) { StartCap = LineCap.Round, EndCap = LineCap.Round };
            using var core = new Pen(A(Color.White, alpha * sa * 0.9f), 3.5f) { StartCap = LineCap.Round, EndCap = LineCap.Round };
            var rect = new RectangleF(hand.X - len, hand.Y - len, len * 2, len * 2);
            g.DrawArc(wide, rect, deg - 78f, 78f);
            g.DrawArc(core, rect, deg - 70f, 70f);
        }

        // front arm to hand
        using (var arm = new Pen(A(pal.Skin, alpha), 5f) { StartCap = LineCap.Round, EndCap = LineCap.Round })
            g.DrawLine(arm, shoulder.X, shoulder.Y + 1, hand.X, hand.Y);

        float len2;
        switch (f.Kind == EnemyKind.None ? (f.Faction == Faction.Player ? 100 : 0) : (int)f.Kind)
        {
            case 100: // player: spear
                len2 = 64f;
                var tip = new PointF(hand.X + dir.X * len2, hand.Y + dir.Y * len2);
                var back = new PointF(hand.X - dir.X * 12f, hand.Y - dir.Y * 12f);
                using (var shaft = new Pen(A(Color.FromArgb(120, 80, 45), alpha), 3f) { StartCap = LineCap.Round, EndCap = LineCap.Round })
                    g.DrawLine(shaft, back.X, back.Y, tip.X, tip.Y);
                using (var headBrush = new SolidBrush(A(pal.Metal, alpha)))
                {
                    var perp = new PointF(-dir.Y, dir.X);
                    var sp = new GraphicsPath();
                    sp.AddPolygon(new[]
                    {
                        new PointF(tip.X + dir.X * 10, tip.Y + dir.Y * 10),
                        new PointF(tip.X + perp.X * 4, tip.Y + perp.Y * 4),
                        new PointF(tip.X - perp.X * 4, tip.Y - perp.Y * 4),
                    });
                    g.FillPath(headBrush, sp);
                    sp.Dispose();
                }
                using (var tassel = new SolidBrush(A(pal.Cloth, alpha)))
                    g.FillEllipse(tassel, hand.X + dir.X * len2 * 0.82f - 3, hand.Y + dir.Y * len2 * 0.82f - 3, 6, 6);
                break;

            case (int)EnemyKind.Brute: // hammer
                len2 = 34f;
                var htip = new PointF(hand.X + dir.X * len2, hand.Y + dir.Y * len2);
                using (var handle = new Pen(A(Color.FromArgb(90, 60, 40), alpha), 4f) { StartCap = LineCap.Round })
                    g.DrawLine(handle, hand.X, hand.Y, htip.X, htip.Y);
                using (var headBrush = new SolidBrush(A(Blend(pal.Metal, Color.Black, 0.25f), alpha)))
                {
                    var perp = new PointF(-dir.Y, dir.X);
                    var hp = new GraphicsPath();
                    hp.AddPolygon(new[]
                    {
                        new PointF(htip.X + perp.X * 12 + dir.X * 8, htip.Y + perp.Y * 12 + dir.Y * 8),
                        new PointF(htip.X - perp.X * 12 + dir.X * 8, htip.Y - perp.Y * 12 + dir.Y * 8),
                        new PointF(htip.X - perp.X * 12 - dir.X * 8, htip.Y - perp.Y * 12 - dir.Y * 8),
                        new PointF(htip.X + perp.X * 12 - dir.X * 8, htip.Y + perp.Y * 12 - dir.Y * 8),
                    });
                    g.FillPath(headBrush, hp);
                    hp.Dispose();
                }
                break;

            default: // grunt: broadsword (dao)
                len2 = 36f;
                var btip = new PointF(hand.X + dir.X * len2, hand.Y + dir.Y * len2);
                using (var guard = new Pen(A(pal.Trim, alpha), 5f))
                {
                    var perp = new PointF(-dir.Y, dir.X);
                    g.DrawLine(guard, hand.X + perp.X * 5, hand.Y + perp.Y * 5, hand.X - perp.X * 5, hand.Y - perp.Y * 5);
                }
                using (var blade = new Pen(A(pal.Metal, alpha), 4f) { EndCap = LineCap.Triangle })
                    g.DrawLine(blade, hand.X, hand.Y, btip.X, btip.Y);
                break;
        }
    }

    /// <summary>Weapon angle in degrees; 0 = forward, negative = up. Drives windup/strike/recovery.</summary>
    private static float WeaponAngle(Fighter f)
    {
        const float rest = -58f, raised = -158f, forward = 46f;
        if (f.State != AnimState.Attack)
            return rest + MathF.Sin(f.AnimClock * 2.5f) * 4f;

        float p = Math.Clamp(f.AttackElapsed / f.AttackDuration, 0f, 1f);
        float wf = f.AttackWindup / f.AttackDuration;
        float af = f.AttackActiveDur / f.AttackDuration;

        if (p < wf)
            return Lerp(rest, raised, EaseOut(p / MathF.Max(0.01f, wf)));
        if (p < wf + af)
            return Lerp(raised, forward, EaseIn((p - wf) / MathF.Max(0.01f, af)));
        return Lerp(forward, rest, (p - wf - af) / MathF.Max(0.01f, 1f - wf - af));
    }

    private static float Lerp(float a, float b, float t) => a + (b - a) * Math.Clamp(t, 0f, 1f);
    private static float EaseIn(float t) => t * t;
    private static float EaseOut(float t) => 1f - (1f - t) * (1f - t);

    private static Color A(Color c, float a) => Color.FromArgb((int)Math.Clamp(c.A * a, 0f, 255f), c.R, c.G, c.B);

    private static Color Blend(Color a, Color b, float t) => Color.FromArgb(
        (int)(a.R + (b.R - a.R) * t),
        (int)(a.G + (b.G - a.G) * t),
        (int)(a.B + (b.B - a.B) * t));
}
