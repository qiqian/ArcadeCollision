using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ArcCollision.Battlefield;

/// <summary>
/// "Battlefield" - a compact arcade beat-'em-up (inspired by Knights of Valour /
/// 街机三国) that stress-tests the ArcCollision library with dozens of jostling
/// bodies, melee swings and swept arrows.
/// </summary>
public sealed class GameForm : Form
{
    private const int OriginX = 10;
    private const int OriginY = 76;

    private readonly Game _game = new();
    private readonly HashSet<Keys> _keys = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly System.Windows.Forms.Timer _timer = new();
    private long _lastTicks;

    public GameForm()
    {
        Text = "ArcCollision — Battlefield";
        ClientSize = new Size((int)Game.ArenaWidth + OriginX * 2, (int)Game.ArenaHeight + OriginY + 12);
        BackColor = Color.FromArgb(16, 18, 22);
        DoubleBuffered = true;
        KeyPreview = true;
        MaximizeBox = false;
        FormBorderStyle = FormBorderStyle.FixedSingle;

        _lastTicks = _clock.ElapsedTicks;
        _timer.Interval = 16;
        _timer.Tick += OnTick;
        _timer.Start();
    }

    // -------------------------------------------------------------- input

    protected override void OnKeyDown(KeyEventArgs e)
    {
        _keys.Add(e.KeyCode);
        if (e.KeyCode == Keys.R && _game.GameOver)
            _game.Reset();
        base.OnKeyDown(e);
    }

    protected override void OnKeyUp(KeyEventArgs e)
    {
        _keys.Remove(e.KeyCode);
        base.OnKeyUp(e);
    }

    private Vec2 ReadMove()
    {
        float x = 0, y = 0;
        if (_keys.Contains(Keys.A) || _keys.Contains(Keys.Left)) x -= 1;
        if (_keys.Contains(Keys.D) || _keys.Contains(Keys.Right)) x += 1;
        if (_keys.Contains(Keys.W) || _keys.Contains(Keys.Up)) y -= 1;
        if (_keys.Contains(Keys.S) || _keys.Contains(Keys.Down)) y += 1;
        return new Vec2(x, y);
    }

    private bool ReadAttack() =>
        _keys.Contains(Keys.J) || _keys.Contains(Keys.Space) || _keys.Contains(Keys.K);

    // --------------------------------------------------------------- loop

    private void OnTick(object? sender, EventArgs e)
    {
        long now = _clock.ElapsedTicks;
        float dt = (float)((now - _lastTicks) / (double)Stopwatch.Frequency);
        _lastTicks = now;
        if (dt > 0.05f) dt = 0.05f; // avoid spiral-of-death on stalls

        _game.SetInput(ReadMove(), ReadAttack());
        _game.Update(dt);
        Invalidate();
    }

    // ------------------------------------------------------------ rendering

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.Clear(BackColor);

        DrawArena(g);

        var state = g.Save();
        g.TranslateTransform(OriginX, OriginY);

        // depth sort: draw far (small y) first
        var order = new List<Fighter>(_game.Fighters);
        order.Sort((a, b) => a.Pos.Y.CompareTo(b.Pos.Y));

        foreach (var f in order)
            DrawShadow(g, f);
        foreach (var f in order)
            DrawFighter(g, f);
        foreach (var p in _game.Projectiles)
            DrawProjectile(g, p);

        g.Restore(state);

        DrawHud(g);
        if (_game.GameOver)
            DrawGameOver(g);
    }

    private void DrawArena(Graphics g)
    {
        var rect = new Rectangle(OriginX, OriginY, (int)Game.ArenaWidth, (int)Game.ArenaHeight);
        using var brush = new LinearGradientBrush(rect,
            Color.FromArgb(58, 74, 52), Color.FromArgb(38, 50, 36), LinearGradientMode.Vertical);
        g.FillRectangle(brush, rect);

        using var stripe = new SolidBrush(Color.FromArgb(18, 255, 255, 255));
        for (int x = 0; x < Game.ArenaWidth; x += 80)
            g.FillRectangle(stripe, OriginX + x, OriginY, 40, (int)Game.ArenaHeight);

        using var border = new Pen(Color.FromArgb(140, 150, 110), 3f);
        g.DrawRectangle(border, rect);
    }

    private static void DrawShadow(Graphics g, Fighter f)
    {
        if (!f.Alive) return;
        using var b = new SolidBrush(Color.FromArgb(70, 0, 0, 0));
        float rx = f.Radius * 1.05f, ry = f.Radius * 0.5f;
        g.FillEllipse(b, f.Pos.X - rx, f.Pos.Y + f.Radius * 0.55f - ry, rx * 2, ry * 2);
    }

    private void DrawFighter(Graphics g, Fighter f)
    {
        if (!f.Alive) return;

        // melee swing arc
        if (f.Attacking)
            DrawSwing(g, f);

        Color body = f.Faction == Faction.Player
            ? Color.FromArgb(90, 170, 255)
            : f.Kind == EnemyKind.Archer ? Color.FromArgb(120, 210, 130) : Color.FromArgb(230, 100, 96);
        if (f.HurtFlash > 0f)
            body = Blend(body, Color.White, 0.6f);

        float r = f.Radius;
        using (var fill = new SolidBrush(body))
            g.FillEllipse(fill, f.Pos.X - r, f.Pos.Y - r, r * 2, r * 2);
        using (var edge = new Pen(Color.FromArgb(30, 30, 40), 2f))
            g.DrawEllipse(edge, f.Pos.X - r, f.Pos.Y - r, r * 2, r * 2);

        // facing marker (a little "weapon" nub)
        Vec2 dir = new(f.Facing, 0f);
        Vec2 tip = f.Pos + dir * (r + 6f);
        using (var pen = new Pen(Color.FromArgb(240, 240, 245), 3f))
            g.DrawLine(pen, f.Pos.X + dir.X * r * 0.3f, f.Pos.Y, tip.X, tip.Y);

        DrawHealthBar(g, f);
    }

    private static void DrawSwing(Graphics g, Fighter f)
    {
        Capsule s = f.Swing();
        Color c = f.AttackActive
            ? Color.FromArgb(180, 255, 240, 140)
            : Color.FromArgb(70, 255, 255, 255);
        using var pen = new Pen(c, f.AttackActive ? 3f : 1.5f);
        Vec2 dir = (s.B - s.A).Normalized(Vec2.UnitX);
        Vec2 n = dir.Perp * s.Radius;
        g.DrawLine(pen, s.A.X + n.X, s.A.Y + n.Y, s.B.X + n.X, s.B.Y + n.Y);
        g.DrawLine(pen, s.A.X - n.X, s.A.Y - n.Y, s.B.X - n.X, s.B.Y - n.Y);
        g.DrawEllipse(pen, s.B.X - s.Radius, s.B.Y - s.Radius, s.Radius * 2, s.Radius * 2);
    }

    private static void DrawHealthBar(Graphics g, Fighter f)
    {
        if (f.Faction == Faction.Enemy && f.Health >= f.MaxHealth)
            return;
        float w = f.Radius * 2f;
        float x = f.Pos.X - f.Radius;
        float y = f.Pos.Y - f.Radius - 10f;
        float pct = Math.Clamp(f.Health / f.MaxHealth, 0f, 1f);
        using var bg = new SolidBrush(Color.FromArgb(180, 20, 20, 20));
        using var fg = new SolidBrush(f.Faction == Faction.Player ? Color.FromArgb(90, 200, 255) : Color.FromArgb(230, 90, 90));
        g.FillRectangle(bg, x, y, w, 4f);
        g.FillRectangle(fg, x, y, w * pct, 4f);
    }

    private static void DrawProjectile(Graphics g, Projectile p)
    {
        if (!p.Alive) return;
        Vec2 dir = p.Vel.Normalized(Vec2.UnitX);
        Vec2 tail = p.Pos - dir * 14f;
        using var pen = new Pen(Color.FromArgb(255, 235, 170), 2.5f);
        g.DrawLine(pen, tail.X, tail.Y, p.Pos.X, p.Pos.Y);
        using var head = new SolidBrush(Color.FromArgb(255, 250, 210));
        g.FillEllipse(head, p.Pos.X - 3f, p.Pos.Y - 3f, 6f, 6f);
    }

    private void DrawHud(Graphics g)
    {
        using var title = new Font("Segoe UI Semibold", 13f, FontStyle.Bold);
        using var body = new Font("Consolas", 10.5f);
        using var white = new SolidBrush(Color.White);
        using var dim = new SolidBrush(Color.FromArgb(190, 210, 220));

        g.DrawString("BATTLEFIELD", title, white, 12, 8);

        // player HP bar
        float hpPct = Math.Clamp(_game.Player.Health / _game.Player.MaxHealth, 0f, 1f);
        var hpRect = new RectangleF(200, 14, 260, 20);
        using (var bg = new SolidBrush(Color.FromArgb(60, 60, 70)))
            g.FillRectangle(bg, hpRect);
        using (var fg = new SolidBrush(Color.FromArgb(90, 200, 255)))
            g.FillRectangle(fg, hpRect.X, hpRect.Y, hpRect.Width * hpPct, hpRect.Height);
        using (var pen = new Pen(Color.FromArgb(150, 160, 170)))
            g.DrawRectangle(pen, hpRect.X, hpRect.Y, hpRect.Width, hpRect.Height);
        g.DrawString($"HP {_game.Player.Health:0}/{_game.Player.MaxHealth:0}", body, white, hpRect.X + 8, hpRect.Y + 2);

        g.DrawString($"SCORE {_game.Score}    WAVE {_game.Wave + 1}", title, white, 500, 10);
        g.DrawString($"broadphase pairs: {_game.BroadphasePairs}   entities: {_game.Fighters.Count}", body, dim, 500, 40);
        g.DrawString("Move: WASD / Arrows    Attack: J / Space", body, dim, 900, 12);
        g.DrawString("Powered by ArcCollision", body, dim, 900, 40);
    }

    private void DrawGameOver(Graphics g)
    {
        using var shade = new SolidBrush(Color.FromArgb(150, 0, 0, 0));
        g.FillRectangle(shade, 0, 0, ClientSize.Width, ClientSize.Height);
        using var big = new Font("Segoe UI", 40f, FontStyle.Bold);
        using var small = new Font("Segoe UI", 14f);
        using var brush = new SolidBrush(Color.White);
        var sz = g.MeasureString("GAME OVER", big);
        g.DrawString("GAME OVER", big, brush, (ClientSize.Width - sz.Width) / 2, ClientSize.Height / 2 - 70);
        string msg = $"Score {_game.Score}   —   press R to fight again";
        var sz2 = g.MeasureString(msg, small);
        g.DrawString(msg, small, brush, (ClientSize.Width - sz2.Width) / 2, ClientSize.Height / 2 + 10);
    }

    private static Color Blend(Color a, Color b, float t) => Color.FromArgb(
        (int)(a.R + (b.R - a.R) * t),
        (int)(a.G + (b.G - a.G) * t),
        (int)(a.B + (b.B - a.B) * t));
}
