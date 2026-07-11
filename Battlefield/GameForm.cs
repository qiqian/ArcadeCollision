using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ArcCollision.Battlefield;

/// <summary>
/// "Battlefield" — an arcade beat-'em-up in the spirit of Knights of Valour /
/// 街机三国. Runs the whole crowd on the ArcCollision library while delivering
/// arcade-style hit feedback: combos, hitstop, screen shake, slash arcs, sparks
/// and floating damage numbers.
/// </summary>
public sealed class GameForm : Form
{
    private const int OriginX = 10;
    private const int OriginY = 78;

    private readonly Game _game = new();
    private readonly HashSet<Keys> _keys = new();
    private readonly HashSet<Keys> _prev = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly System.Windows.Forms.Timer _timer = new();
    private readonly Random _rng = new();
    private long _lastTicks;
    private float _titleFade = 3.0f;

    private readonly Font _fBig = new("Segoe UI", 40f, FontStyle.Bold);
    private readonly Font _fTitle = new("Segoe UI Semibold", 16f, FontStyle.Bold);
    private readonly Font _fHud = new("Consolas", 11f, FontStyle.Bold);
    private readonly Font _fCombo = new("Segoe UI", 30f, FontStyle.Bold);
    private readonly Font _fDmg = new("Segoe UI", 12f, FontStyle.Bold);
    private readonly Font _fDmgCrit = new("Segoe UI", 18f, FontStyle.Bold);

    public GameForm()
    {
        Text = "ArcCollision — Battlefield 三国";
        ClientSize = new Size((int)Game.ArenaWidth + OriginX * 2, (int)Game.ArenaHeight + OriginY + 12);
        BackColor = Color.FromArgb(12, 12, 16);
        DoubleBuffered = true;
        KeyPreview = true;
        MaximizeBox = false;
        FormBorderStyle = FormBorderStyle.FixedSingle;

        _lastTicks = _clock.ElapsedTicks;
        _timer.Interval = 15;
        _timer.Tick += OnTick;
        _timer.Start();
    }

    // -------------------------------------------------------------- input

    protected override void OnKeyDown(KeyEventArgs e)
    {
        _keys.Add(e.KeyCode);
        if (e.KeyCode == Keys.R && _game.GameOver)
        {
            _game.Reset();
            _titleFade = 1.2f;
        }
        base.OnKeyDown(e);
    }

    protected override void OnKeyUp(KeyEventArgs e)
    {
        _keys.Remove(e.KeyCode);
        base.OnKeyUp(e);
    }

    private bool Down(Keys k) => _keys.Contains(k);
    private bool Pressed(Keys k) => _keys.Contains(k) && !_prev.Contains(k);

    private Vec2 ReadMove()
    {
        float x = 0, y = 0;
        if (Down(Keys.A) || Down(Keys.Left)) x -= 1;
        if (Down(Keys.D) || Down(Keys.Right)) x += 1;
        if (Down(Keys.W) || Down(Keys.Up)) y -= 1;
        if (Down(Keys.S) || Down(Keys.Down)) y += 1;
        return new Vec2(x, y);
    }

    // --------------------------------------------------------------- loop

    private void OnTick(object? sender, EventArgs e)
    {
        long now = _clock.ElapsedTicks;
        float dt = (float)((now - _lastTicks) / (double)Stopwatch.Frequency);
        _lastTicks = now;
        if (dt > 0.05f) dt = 0.05f;
        if (_titleFade > 0f) _titleFade -= dt;

        bool attack = Pressed(Keys.J) || Pressed(Keys.Z);
        bool heavy = Pressed(Keys.K) || Pressed(Keys.X);
        bool dash = Pressed(Keys.Space) || Pressed(Keys.L) || Pressed(Keys.ShiftKey);

        _game.SetInput(ReadMove(), attack, heavy, dash);
        _game.Update(dt);

        _prev.Clear();
        _prev.UnionWith(_keys);
        Invalidate();
    }

    // ------------------------------------------------------------ rendering

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.Clear(BackColor);

        float sx = 0, sy = 0;
        if (_game.ShakeMag > 0.1f)
        {
            sx = (float)(_rng.NextDouble() - 0.5) * _game.ShakeMag * 2f;
            sy = (float)(_rng.NextDouble() - 0.5) * _game.ShakeMag * 2f;
        }

        GraphicsState world = g.Save();
        g.TranslateTransform(OriginX + sx, OriginY + sy);
        g.SetClip(new RectangleF(-OriginX, -OriginY, ClientSize.Width, ClientSize.Height));

        DrawBackground(g);

        foreach (var d in _game.Dusts)
            DrawDust(g, d);

        foreach (var f in _game.Fighters)
            CharacterArt.DrawShadow(g, f);

        var order = new List<Fighter>(_game.Fighters);
        order.Sort((a, b) => a.Pos.Y.CompareTo(b.Pos.Y));
        foreach (var f in order)
            CharacterArt.Draw(g, f);

        foreach (var s in _game.Sparks)
            DrawSpark(g, s);
        foreach (var n in _game.Numbers)
            DrawNumber(g, n);

        g.ResetClip();
        g.Restore(world);

        if (_game.FlashFx > 0.01f)
            using (var fl = new SolidBrush(Color.FromArgb((int)(90 * Math.Clamp(_game.FlashFx, 0f, 1f)), 255, 255, 255)))
                g.FillRectangle(fl, 0, 0, ClientSize.Width, ClientSize.Height);

        DrawHud(g);
        if (_titleFade > 0f)
            DrawTitle(g);
        if (_game.GameOver)
            DrawGameOver(g);
    }

    // ---- world ----

    private void DrawBackground(Graphics g)
    {
        var arena = new RectangleF(0, 0, Game.ArenaWidth, Game.ArenaHeight);

        // back wall
        using (var wall = new LinearGradientBrush(
            new RectangleF(0, 0, Game.ArenaWidth, Game.FloorTop),
            Color.FromArgb(46, 40, 54), Color.FromArgb(30, 28, 40), LinearGradientMode.Vertical))
            g.FillRectangle(wall, 0, 0, Game.ArenaWidth, Game.FloorTop);

        // hanging banners (三国 vibe)
        Color[] bcol = { Color.FromArgb(180, 60, 60), Color.FromArgb(60, 90, 180), Color.FromArgb(200, 170, 70) };
        for (int i = 0; i < 7; i++)
        {
            float x = 90 + i * 165;
            Color c = bcol[i % bcol.Length];
            using var b = new SolidBrush(Color.FromArgb(220, c));
            g.FillPolygon(b, new[]
            {
                new PointF(x - 16, 0), new PointF(x + 16, 0),
                new PointF(x + 16, Game.FloorTop - 18), new PointF(x, Game.FloorTop - 8),
                new PointF(x - 16, Game.FloorTop - 18),
            });
            using var em = new SolidBrush(Color.FromArgb(230, 235, 220));
            g.FillEllipse(em, x - 7, 22, 14, 14);
        }

        // ground
        using (var ground = new LinearGradientBrush(
            new RectangleF(0, Game.FloorTop, Game.ArenaWidth, Game.ArenaHeight - Game.FloorTop),
            Color.FromArgb(96, 84, 64), Color.FromArgb(66, 58, 46), LinearGradientMode.Vertical))
            g.FillRectangle(ground, 0, Game.FloorTop, Game.ArenaWidth, Game.ArenaHeight - Game.FloorTop);

        // perspective floor lines
        using (var line = new Pen(Color.FromArgb(30, 0, 0, 0), 2f))
        {
            for (float y = Game.FloorTop + 40; y < Game.ArenaHeight; y += 46)
                g.DrawLine(line, 0, y, Game.ArenaWidth, y);
        }

        using (var edge = new Pen(Color.FromArgb(20, 0, 0, 0), 6f))
            g.DrawLine(edge, 0, Game.FloorTop, Game.ArenaWidth, Game.FloorTop);
        using (var border = new Pen(Color.FromArgb(120, 100, 70), 3f))
            g.DrawRectangle(border, arena.X, arena.Y, arena.Width, arena.Height);
    }

    private static void DrawDust(Graphics g, Dust d)
    {
        float t = d.Age / d.Life;
        int a = (int)(120 * (1f - t));
        if (a <= 0) return;
        float r = d.Size * (0.6f + t);
        using var b = new SolidBrush(Color.FromArgb(a, 170, 155, 130));
        g.FillEllipse(b, d.Pos.X - r, d.Pos.Y - r, r * 2, r * 2);
    }

    private static void DrawSpark(Graphics g, HitSpark s)
    {
        float t = s.Age / s.Life;
        float k = 1f - t;
        if (k <= 0) return;

        GraphicsState gs = g.Save();
        g.TranslateTransform(s.Pos.X, s.Pos.Y);
        g.RotateTransform(s.Angle * 180f / MathF.PI + s.Age * 200f);
        float rad = (s.Heavy ? 26f : 15f) * s.Size * (0.5f + t);

        Color col = s.Heavy ? Color.FromArgb((int)(255 * k), 255, 180, 90) : Color.FromArgb((int)(255 * k), 255, 245, 200);
        using (var pen = new Pen(col, s.Heavy ? 3.5f : 2.2f) { StartCap = LineCap.Round, EndCap = LineCap.Round })
        {
            int spokes = s.Heavy ? 8 : 6;
            for (int i = 0; i < spokes; i++)
            {
                double ang = i * Math.PI * 2 / spokes;
                float ix = (float)Math.Cos(ang) * rad * 0.35f;
                float iy = (float)Math.Sin(ang) * rad * 0.35f;
                float ox = (float)Math.Cos(ang) * rad;
                float oy = (float)Math.Sin(ang) * rad;
                g.DrawLine(pen, ix, iy, ox, oy);
            }
        }
        using (var core = new SolidBrush(Color.FromArgb((int)(255 * k), 255, 255, 255)))
            g.FillEllipse(core, -rad * 0.28f, -rad * 0.28f, rad * 0.56f, rad * 0.56f);
        g.Restore(gs);
    }

    private void DrawNumber(Graphics g, DamageNumber n)
    {
        float t = n.Age / n.Life;
        int a = (int)(255 * (1f - t * t));
        Font font = n.Crit ? _fDmgCrit : _fDmg;
        string text = n.Value.ToString();
        Color fill = n.Crit ? Color.FromArgb(a, 255, 190, 70) : Color.FromArgb(a, 255, 245, 180);
        using var outline = new SolidBrush(Color.FromArgb(a, 20, 15, 10));
        using var brush = new SolidBrush(fill);
        // cheap outline
        g.DrawString(text, font, outline, n.Pos.X + 1, n.Pos.Y + 1);
        g.DrawString(text, font, brush, n.Pos.X, n.Pos.Y);
    }

    // ---- HUD ----

    private void DrawHud(Graphics g)
    {
        using var white = new SolidBrush(Color.White);
        using var dim = new SolidBrush(Color.FromArgb(200, 210, 220));

        // player plate
        g.DrawString("赵云  ZHAO YUN", _fTitle, white, 14, 10);
        var hp = new RectangleF(16, 40, 300, 20);
        float pct = Math.Clamp(_game.Player.Health / _game.Player.MaxHealth, 0f, 1f);
        using (var bg = new SolidBrush(Color.FromArgb(60, 30, 30)))
            g.FillRectangle(bg, hp);
        using (var fg = new LinearGradientBrush(hp, Color.FromArgb(120, 220, 255), Color.FromArgb(60, 140, 230), LinearGradientMode.Vertical))
            g.FillRectangle(fg, hp.X, hp.Y, hp.Width * pct, hp.Height);
        using (var pen = new Pen(Color.FromArgb(180, 190, 200), 1.5f))
            g.DrawRectangle(pen, hp.X, hp.Y, hp.Width, hp.Height);

        // lives
        for (int i = 0; i < _game.Lives; i++)
            DrawLifeIcon(g, 330 + i * 22, 44);

        // wave + score
        string wave = $"WAVE {_game.Wave + 1}";
        var wsz = g.MeasureString(wave, _fTitle);
        g.DrawString(wave, _fTitle, white, (ClientSize.Width - wsz.Width) / 2, 12);
        string score = $"SCORE {_game.Score:00000}";
        var ssz = g.MeasureString(score, _fTitle);
        g.DrawString(score, _fTitle, white, ClientSize.Width - ssz.Width - 16, 12);
        string best = $"best combo x{_game.BestCombo}";
        var bsz = g.MeasureString(best, _fHud);
        g.DrawString(best, _fHud, dim, ClientSize.Width - bsz.Width - 16, 44);

        // combo meter
        if (_game.Combo >= 2)
        {
            float pop = 1f + Math.Clamp((_game.ComboTimer - 1.9f) / 0.3f, 0f, 1f) * 0.5f;
            string c = $"{_game.Combo} HIT";
            var st = g.Save();
            float cx = ClientSize.Width / 2f;
            float cy = OriginY + 34;
            g.TranslateTransform(cx, cy);
            g.ScaleTransform(pop, pop);
            var csz = g.MeasureString(c, _fCombo);
            using var shadow = new SolidBrush(Color.FromArgb(160, 0, 0, 0));
            using var combo = new SolidBrush(_game.Combo >= 10 ? Color.FromArgb(255, 120, 90) : Color.FromArgb(255, 220, 120));
            g.DrawString(c, _fCombo, shadow, -csz.Width / 2 + 2, 2);
            g.DrawString(c, _fCombo, combo, -csz.Width / 2, 0);
            g.Restore(st);
        }

        using var hint = new SolidBrush(Color.FromArgb(150, 200, 210));
        g.DrawString("Move WASD / Arrows    Attack J   Heavy K   Dash Space",
            _fHud, hint, 16, ClientSize.Height - 24);
    }

    private static void DrawLifeIcon(Graphics g, float x, float y)
    {
        using var b = new SolidBrush(Color.FromArgb(230, 90, 90));
        var p = new GraphicsPath();
        p.AddPolygon(new[]
        {
            new PointF(x, y + 12), new PointF(x - 8, y + 3),
            new PointF(x - 8, y - 2), new PointF(x - 4, y - 6),
            new PointF(x, y - 2), new PointF(x + 4, y - 6),
            new PointF(x + 8, y - 2), new PointF(x + 8, y + 3),
        });
        g.FillPath(b, p);
        p.Dispose();
    }

    private void DrawTitle(Graphics g)
    {
        int a = (int)(255 * Math.Clamp(_titleFade / 1.2f, 0f, 1f));
        using var brush = new SolidBrush(Color.FromArgb(a, 255, 230, 150));
        using var sub = new SolidBrush(Color.FromArgb(a, 220, 220, 230));
        var sz = g.MeasureString("BATTLEFIELD", _fBig);
        g.DrawString("BATTLEFIELD", _fBig, brush, (ClientSize.Width - sz.Width) / 2, ClientSize.Height * 0.36f);
        string s = "powered by ArcCollision — defeat the endless army!";
        var sz2 = g.MeasureString(s, _fTitle);
        g.DrawString(s, _fTitle, sub, (ClientSize.Width - sz2.Width) / 2, ClientSize.Height * 0.36f + 58);
    }

    private void DrawGameOver(Graphics g)
    {
        using var shade = new SolidBrush(Color.FromArgb(170, 0, 0, 0));
        g.FillRectangle(shade, 0, 0, ClientSize.Width, ClientSize.Height);
        using var brush = new SolidBrush(Color.White);
        var sz = g.MeasureString("GAME OVER", _fBig);
        g.DrawString("GAME OVER", _fBig, brush, (ClientSize.Width - sz.Width) / 2, ClientSize.Height / 2f - 80);
        string msg = $"Score {_game.Score}    Best Combo x{_game.BestCombo}    Waves {_game.Wave + 1}";
        var sz2 = g.MeasureString(msg, _fTitle);
        g.DrawString(msg, _fTitle, brush, (ClientSize.Width - sz2.Width) / 2, ClientSize.Height / 2f);
        string r = "press R to fight again";
        var sz3 = g.MeasureString(r, _fTitle);
        g.DrawString(r, _fTitle, brush, (ClientSize.Width - sz3.Width) / 2, ClientSize.Height / 2f + 40);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _timer.Dispose();
            _fBig.Dispose();
            _fTitle.Dispose();
            _fHud.Dispose();
            _fCombo.Dispose();
            _fDmg.Dispose();
            _fDmgCrit.Dispose();
        }
        base.Dispose(disposing);
    }
}
