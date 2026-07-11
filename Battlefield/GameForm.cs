using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.IO;
using System.Windows.Forms;

namespace ArcCollision.Battlefield;

/// <summary>
/// Renders and drives the Battlefield beat-'em-up. Combat behaviour is a port of
/// the Quiver "Beat 'Em Up" Godot template; the art is that template's chad and
/// sarge sprites, animated frame-by-frame here with GDI+.
/// </summary>
public sealed class GameForm : Form
{
    private readonly Game _game = new();
    private readonly HashSet<Keys> _keys = new();
    private readonly HashSet<Keys> _prev = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly System.Windows.Forms.Timer _timer = new();
    private long _lastTicks;
    private double _accumulator;
    private bool _attackQueued, _jumpQueued;
    private float _presentationTime;
    private float _camX;
    private float _camY;
    private float _camZoom = 1f;
    private bool _camReady;
    private float _limitLeft, _limitTop, _limitRight, _limitBottom;
    private float _fromLeft, _fromTop, _fromRight, _fromBottom, _fromZoom;
    private float _toLeft, _toTop, _toRight, _toBottom, _toZoom;
    private float _cameraTransitionAge = .8f;
    private readonly StageRenderer _stageRenderer = new();
    private bool _taxDeathActive;
    private float _taxDeathAge;
    private Vec2 _taxDeathPos;
    private float _taxDeathFacing = -1f;

    private const float VW = Game.ArenaW;   // virtual render width
    private const float VH = 720f;          // virtual render height

    private readonly Font _fEndTitle = new("Segoe UI", 44f, FontStyle.Bold);
    private readonly Font _fEndHint = new("Segoe UI", 16f, FontStyle.Bold);
    private readonly Bitmap _buffer = new((int)VW, (int)VH, PixelFormat.Format32bppPArgb);
    private Direct2DPresenter? _presenter;
    private const double FixedStep = 1.0 / 60.0;

    private readonly Image? _hudPlayerBase, _hudPlayerUnder, _hudPlayerProgress, _hudHeart;
    private readonly Image? _hudEnemyBase, _hudEnemyUnder, _hudEnemyProgress, _hudEnemyHeart;
    private readonly Image? _profileChad, _profileSarge, _profileTaxman;

    public GameForm()
    {
        Text = "Template Beat Project — Battlefield";
        AutoScaleMode = AutoScaleMode.None;
        BackColor = Color.FromArgb(12, 12, 16);
        DoubleBuffered = false;
        SetStyle(ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint
            | ControlStyles.Opaque, true);
        KeyPreview = true;
        MaximizeBox = false;
        FormBorderStyle = FormBorderStyle.FixedSingle;

        // Godot's 1920x1080 viewport is uniformly scaled by 2/3 to 1280x720.
        var wa = Screen.PrimaryScreen?.WorkingArea ?? new Rectangle(0, 0, 1280, 720);
        float ar = VW / VH;
        int cw = Math.Min(1280, (int)(wa.Width * 0.94f));
        int ch = Math.Min((int)(wa.Height * 0.90f), (int)(cw / ar));
        cw = (int)(ch * ar);
        ClientSize = new Size(cw, ch);
        StartPosition = FormStartPosition.CenterScreen;

        string assets = Path.Combine(AppContext.BaseDirectory, "Assets");
        string playerHud = Path.Combine(assets, "ui", "gameplay_hud", "player_health_bar", "pngs");
        string enemyHud = Path.Combine(assets, "ui", "gameplay_hud", "enemy_health_bar", "pngs");
        _hudPlayerBase = TryLoad(Path.Combine(playerHud, "health_bar_base.png"));
        _hudPlayerUnder = TryLoad(Path.Combine(playerHud, "health_bar_under.png"));
        _hudPlayerProgress = TryLoad(Path.Combine(playerHud, "health_bar_progress.png"));
        _hudHeart = TryLoad(Path.Combine(playerHud, "heart_icon.png"));
        _hudEnemyBase = TryLoad(Path.Combine(enemyHud, "small_health_bar_base.png"));
        _hudEnemyUnder = TryLoad(Path.Combine(enemyHud, "small_health_bar_under.png"));
        _hudEnemyProgress = TryLoad(Path.Combine(enemyHud, "small_health_bar_progress.png"));
        _hudEnemyHeart = TryLoad(Path.Combine(enemyHud, "small_heart_icon.png"));
        _profileChad = TryLoad(Path.Combine(assets, "characters", "playable", "chad", "resources", "sprites", "chad_profile.png"));
        _profileSarge = TryLoad(Path.Combine(assets, "characters", "enemies", "sargent", "resources", "sprites", "health_bar_revised_all_sarge.png"));
        _profileTaxman = TryLoad(Path.Combine(assets, "characters", "enemies", "tax_man", "resources", "sprites", "health_bar_revised_all_tax_man.png"));

        _lastTicks = _clock.ElapsedTicks;
        _timer.Interval = 8;
        _timer.Tick += OnTick;
        _timer.Start();
    }

    private static Image? TryLoad(string p) => File.Exists(p) ? Image.FromFile(p) : null;

    protected override void OnHandleCreated(EventArgs e)
    {
        base.OnHandleCreated(e);
        if (DesignMode) return;
        try
        {
            _presenter = new Direct2DPresenter(Handle, ClientSize);
            Text = "Template Beat Project — Battlefield (Direct2D/D3D)";
        }
        catch
        {
            // Remote sessions or unavailable graphics drivers can still use
            // the software buffer without preventing the game from starting.
            _presenter = null;
            Text = "Template Beat Project — Battlefield (software fallback)";
        }
    }

    protected override void OnResize(EventArgs e)
    {
        base.OnResize(e);
        _presenter?.Resize(ClientSize);
    }

    protected override void OnHandleDestroyed(EventArgs e)
    {
        _presenter?.Dispose();
        _presenter = null;
        base.OnHandleDestroyed(e);
    }

    // -------------------------------------------------------------- input

    protected override void OnKeyDown(KeyEventArgs e)
    {
        _keys.Add(e.KeyCode);
        base.OnKeyDown(e);
    }

    protected override void OnKeyUp(KeyEventArgs e) { _keys.Remove(e.KeyCode); base.OnKeyUp(e); }

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
        double elapsed = (now - _lastTicks) / (double)Stopwatch.Frequency;
        _lastTicks = now;
        elapsed = Math.Min(elapsed, .25);

        if (Pressed(Keys.R))
        {
            _game.Reset();
            _camReady = false;
            _camZoom = 1f;
            _presentationTime = 0f;
            _taxDeathActive = false;
            _taxDeathAge = 0f;
            _accumulator = 0;
            _attackQueued = _jumpQueued = false;
        }

        _attackQueued |= Pressed(Keys.J);
        _jumpQueued |= Pressed(Keys.Space);
        _accumulator += elapsed;
        int steps = 0;
        while (_accumulator >= FixedStep && steps < 15)
        {
            bool attack = _attackQueued;
            bool jump = _jumpQueued;
            _attackQueued = _jumpQueued = false;
            _game.SetInput(ReadMove(), attack, jump);
            _game.Update((float)FixedStep);
            UpdatePresentation((float)FixedStep);
            UpdateCamera((float)FixedStep);
            _accumulator -= FixedStep;
            steps++;
        }

        _prev.Clear();
        _prev.UnionWith(_keys);
        Invalidate();
    }

    private void UpdatePresentation(float dt)
    {
        _presentationTime += dt;
        Fighter? taxman = null;
        foreach (Fighter fighter in _game.Fighters)
            if (fighter.Def.SpriteSet == "taxman") { taxman = fighter; break; }

        if (taxman != null && taxman.State == FState.Dead)
        {
            _taxDeathActive = true;
            _taxDeathAge = taxman.StateTime;
            _taxDeathPos = taxman.Pos;
            _taxDeathFacing = taxman.Facing;
        }
        else if (_taxDeathActive)
            _taxDeathAge += dt;
    }

    // ------------------------------------------------------------ rendering

    protected override void OnPaint(PaintEventArgs e)
    {
        RenderBuffer();
        if (_presenter != null)
            _presenter.Present(_buffer, ClientSize);
        else
            e.Graphics.DrawImage(_buffer, ClientRectangle);
    }

    internal void SaveScreenshot(string path)
    {
        RenderBuffer();
        _buffer.Save(path, ImageFormat.Png);
    }

    internal bool VerifyHardwarePresenter()
    {
        _ = Handle; // force HWND creation and Direct2D initialization
        if (_presenter == null) return false;
        RenderBuffer();
        _presenter.Present(_buffer, ClientSize);
        return true;
    }

    private void RenderBuffer()
    {
        using var g = Graphics.FromImage(_buffer);
        g.CompositingQuality = CompositingQuality.HighSpeed;
        g.InterpolationMode = InterpolationMode.NearestNeighbor;
        g.PixelOffsetMode = PixelOffsetMode.Half;
        g.SmoothingMode = SmoothingMode.None;
        RenderScene(g);
    }

    private void RenderScene(Graphics g)
    {
        g.Clear(BackColor);

        float camX = _camReady ? _camX : CameraTargetX();
        float camY = _camReady ? _camY : CameraTargetY();

        GraphicsState world = g.Save();
        float zoom = _camZoom;
        using (var matrix = new Matrix(zoom, 0f, 0f, zoom,
                   -camX * zoom, -camY * zoom))
            g.Transform = matrix;

        _stageRenderer.DrawBackground(g, camX, zoom);
        DrawSeatedTaxman(g);

        GraphicsState combatSpace = g.Save();
        foreach (var f in _game.Fighters) DrawShadow(g, f);

        var order = new List<Fighter>(_game.Fighters);
        order.Sort((a, b) => a.Pos.Y.CompareTo(b.Pos.Y));
        g.SmoothingMode = SmoothingMode.AntiAlias;
        foreach (var f in order)
        {
            DrawTaxVfx(g, f, behindBody: true);
            bool engaging = f.Def.SpriteSet == "taxman" && f.TaxSeat == TaxSeatState.Engage;
            if (engaging)
                DrawClipAt(g, SpriteLibrary.Get("taxman")["seated_engage"],
                    MathF.Max(0f, 1.75f - f.TaxSeatTimer), f.Pos.X, f.Pos.Y - f.SkinY,
                    f.Facing, f.Def.RenderScale);
            else if (!(f.Def.SpriteSet == "taxman" && f.State == FState.Dead))
                DrawFighter(g, f);
            DrawTaxVfx(g, f, behindBody: false);
        }
        DrawTaxDeath(g);

        g.Restore(combatSpace);
        _stageRenderer.DrawForeground(g);

        g.Restore(world);

        DrawHud(g);
        if (_game.GameOver) DrawEndScreen(g);
    }

    private void DrawEndScreen(Graphics g)
    {
        using var shade = new SolidBrush(Color.FromArgb(170, 0, 0, 0));
        g.FillRectangle(shade, 0, 0, VW, VH);

        string title = _game.Win ? "STAGE CLEAR!" : "GAME OVER";
        using var titleBrush = new SolidBrush(_game.Win
            ? Color.FromArgb(255, 226, 130) : Color.White);
        var titleSize = g.MeasureString(title, _fEndTitle);
        g.DrawString(title, _fEndTitle, titleBrush, (VW - titleSize.Width) / 2f, VH / 2f - 100f);

        // Blink so the restart hint is unmissable.
        int alpha = (int)(180 + 75 * MathF.Sin(_presentationTime * 5f));
        using var hintBrush = new SolidBrush(Color.FromArgb(Math.Clamp(alpha, 0, 255), 235, 235, 245));
        string hint = "PRESS  R  TO  RESTART";
        var hintSize = g.MeasureString(hint, _fEndHint);
        g.DrawString(hint, _fEndHint, hintBrush, (VW - hintSize.Width) / 2f, VH / 2f + 4f);
    }

    private float CameraTargetX()
    {
        float visibleWidth = VW / MathF.Max(0.1f, _camZoom);
        float min = _camReady ? _limitLeft : _game.CamMinX;
        float max = _camReady ? _limitRight : _game.CamMaxX;
        float camMax = Math.Max(min, max - visibleWidth);
        return Math.Clamp(_game.Player.Pos.X - visibleWidth / 2f, min, camMax);
    }

    private float CameraTargetY()
    {
        float visibleHeight = VH / MathF.Max(0.1f, _camZoom);
        float min = _camReady ? _limitTop : _game.CamMinY;
        float max = _camReady ? _limitBottom : _game.CamMaxY;
        float camMax = Math.Max(min, max - visibleHeight);
        float cameraCenter = _game.Player.Pos.Y - 100f * Game.WorldScale;
        return Math.Clamp(cameraCenter - visibleHeight / 2f, min, camMax);
    }

    private void UpdateCamera(float dt)
    {
        if (!_camReady)
        {
            _limitLeft = _toLeft = _game.CamMinX;
            _limitTop = _toTop = _game.CamMinY;
            _limitRight = _toRight = _game.CamMaxX;
            _limitBottom = _toBottom = _game.CamMaxY;
            _camZoom = _toZoom = _game.CameraZoom;
            _fromLeft = _limitLeft;
            _fromTop = _limitTop;
            _fromRight = _limitRight;
            _fromBottom = _limitBottom;
            _fromZoom = _camZoom;
            _cameraTransitionAge = .8f;
            _camReady = true;
        }

        if (_toLeft != _game.CamMinX || _toTop != _game.CamMinY
            || _toRight != _game.CamMaxX || _toBottom != _game.CamMaxY
            || _toZoom != _game.CameraZoom)
        {
            _fromLeft = _limitLeft;
            _fromTop = _limitTop;
            _fromRight = _limitRight;
            _fromBottom = _limitBottom;
            _fromZoom = _camZoom;
            _toLeft = _game.CamMinX;
            _toTop = _game.CamMinY;
            _toRight = _game.CamMaxX;
            _toBottom = _game.CamMaxY;
            _toZoom = _game.CameraZoom;
            _cameraTransitionAge = 0f;
        }

        _cameraTransitionAge = MathF.Min(.8f, _cameraTransitionAge + dt);
        float t = _cameraTransitionAge / .8f;
        float eased = t < .5f ? 2f * t * t : 1f - MathF.Pow(-2f * t + 2f, 2f) * .5f;
        _limitLeft = Lerp(_fromLeft, _toLeft, eased);
        _limitTop = Lerp(_fromTop, _toTop, eased);
        _limitRight = Lerp(_fromRight, _toRight, eased);
        _limitBottom = Lerp(_fromBottom, _toBottom, eased);
        _camZoom = Lerp(_fromZoom, _toZoom, eased);

        // Camera2D is a child of Chad, so following is immediate. Only room
        // limits and zoom are tweened by QuiverLevelCamera.delimitate_room.
        _camX = CameraTargetX();
        _camY = CameraTargetY();
    }

    private static float Lerp(float a, float b, float t) => a + (b - a) * t;

    // ---- sprite drawing ----

    private static (string clip, int frame) SelectClip(Fighter f)
    {
        // Turn-around plays over idle/walk while the turn timer is active.
        if (f.TurnTimer > 0f && (f.State == FState.Idle || f.State == FState.Walk))
            return ("turn", -1);

        switch (f.State)
        {
            case FState.Walk: return ("walk", -1);
            case FState.Jump:
                {
                    var set = SpriteLibrary.Get(f.Def.SpriteSet);
                    float impulseLength = set["jump"].Frames.Length / set["jump"].Fps;
                    if (f.StateTime < impulseLength) return ("jump", -1);
                    if (f.SkinVelY > 0f) return ("rising", -1);
                    if (f.SkinY < 90f && set.ContainsKey("landing")) return ("landing", -1);
                    return ("falling", -1);
                }
            case FState.Attack when f.Attack != null:
            case FState.AirAttack when f.Attack != null:
                {
                    var clip = SpriteLibrary.Get(f.Def.SpriteSet)[f.Attack.Clip];
                    int fr = clip.FrameAt(f.AttackTime);
                    return (f.Attack.Clip, Math.Clamp(fr, 0, clip.Frames.Length - 1));
                }
            case FState.Hurt:
                if (f.Def.SpriteSet == "taxman" && !f.Alive) return ("hurt_knockout", 0);
                return (f.HurtHigh ? "hurt_high" : "hurt_mid", 0);
            case FState.Ko:
                if (f.StateTime < 0.09f) return ("ko_launch", 0);
                if (f.Bounces > 0 && f.SkinVelY > 0f && f.SkinY < 130f) return ("ko_bounce", -1);
                return (f.SkinVelY > 60f ? "ko_rising" : "ko_falling", 0);
            case FState.Landed: return ("ko_landed", -1);
            case FState.GetUp: return ("getup", -1);
            case FState.Dead: return ("death", -1);
            default: return ("idle", -1);
        }
    }

    private static void DrawFighter(Graphics g, Fighter f)
    {
        var clips = SpriteLibrary.Get(f.Def.SpriteSet);
        (string name, int frame) = SelectClip(f);
        if (!clips.TryGetValue(name, out var clip)) clip = clips["idle"];
        int idx = frame >= 0 ? frame : clip.FrameAt(f.StateTime);
        idx = Math.Clamp(idx, 0, clip.Frames.Length - 1);
        Bitmap bmp = clip.Frames[idx];

        float S = f.Def.RenderScale;
        float feetX = f.Pos.X;
        float feetY = f.Pos.Y - f.SkinY;
        float w = bmp.Width * S, h = bmp.Height * S;
        (float ox, float oy) = clip.OffsetAt(name.StartsWith("attack") ? f.AttackTime : f.StateTime);
        float cx = clip.BottomAnchor ? feetX : feetX + ox * S * f.Facing;
        float cy = clip.BottomAnchor ? feetY - h / 2f : feetY + oy * S;

        float alpha = 1f;
        if (f.Dead) alpha = Math.Clamp(f.DeathTimer / 0.4f, 0f, 1f);
        if (!f.Dead && f.Invuln > 0f && f.Faction == Faction.Player)
            alpha *= MathF.Sin(f.StateTime * 40f) > 0 ? 1f : 0.4f;

        GraphicsState st = g.Save();
        g.TranslateTransform(cx, cy);
        g.ScaleTransform(clip.FacesLeft ? -f.Facing : f.Facing, 1f);

        var dest = new RectangleF(-w / 2f, -h / 2f, w, h);
        if (alpha >= 0.999f)
            g.DrawImage(bmp, dest.X, dest.Y, dest.Width, dest.Height);
        else
            DrawImageAlpha(g, bmp, dest, alpha);

        // white hit flash
        if (f.HurtFlash > 0f)
            DrawImageWhite(g, bmp, dest, Math.Clamp(f.HurtFlash / 0.14f, 0f, 1f) * 0.75f);

        g.Restore(st);
    }

    private void DrawSeatedTaxman(Graphics g)
    {
        foreach (Fighter fighter in _game.Fighters)
            if (fighter.Def.SpriteSet == "taxman") return;

        Fighter taxman = _game.TaxMan;
        var clips = SpriteLibrary.Get("taxman");
        string name;
        float time;
        switch (taxman.TaxSeat)
        {
            case TaxSeatState.Reveal:
                name = "seated_reveal";
                time = MathF.Max(0f, 2.625f - taxman.TaxSeatTimer);
                break;
            case TaxSeatState.Drink:
                name = "seated_drink";
                time = MathF.Max(0f, 1.625f - taxman.TaxSeatTimer);
                break;
            case TaxSeatState.Laugh:
                name = "seated_laugh";
                time = MathF.Max(0f, 4.08334f - taxman.TaxSeatTimer);
                break;
            default:
                name = "seated_swirl";
                time = _presentationTime;
                break;
        }

        DrawClipAt(g, clips[name], time, taxman.Pos.X, taxman.Pos.Y,
            taxman.Facing, taxman.Def.RenderScale);
    }

    private static void DrawClipAt(Graphics g, Clip clip, float time, float feetX, float feetY,
        float facing, float renderScale, float imageScaleMultiplier = 1f)
    {
        int idx = Math.Clamp(clip.FrameAt(Math.Max(0f, time)), 0, clip.Frames.Length - 1);
        Bitmap bmp = clip.Frames[idx];
        (float ox, float oy) = clip.OffsetAt(Math.Max(0f, time));
        float cx = feetX + ox * renderScale * facing;
        float cy = feetY + oy * renderScale;
        float w = bmp.Width * renderScale * imageScaleMultiplier;
        float h = bmp.Height * renderScale * imageScaleMultiplier;
        GraphicsState state = g.Save();
        g.TranslateTransform(cx, cy);
        g.ScaleTransform(clip.FacesLeft ? -facing : facing, 1f);
        g.DrawImage(bmp, -w / 2f, -h / 2f, w, h);
        g.Restore(state);
    }

    private static void DrawTaxVfx(Graphics g, Fighter fighter, bool behindBody)
    {
        if (fighter.Def.SpriteSet != "taxman" || fighter.Attack?.Clip != "attack_area") return;
        float t = fighter.AttackTime;
        var clips = SpriteLibrary.Get("taxman");

        if (behindBody)
        {
            if (t >= 2f && t < 2.66667f)
                DrawClipAt(g, clips["vfx_area_back"], t - 2f, fighter.Pos.X, fighter.Pos.Y,
                    fighter.Facing, fighter.Def.RenderScale, 2f);
            if (t >= 2.54167f && t < 2.70833f)
                DrawClipAt(g, clips["vfx_area_ground"], t - 2.54167f, fighter.Pos.X, fighter.Pos.Y,
                    fighter.Facing, fighter.Def.RenderScale, 2f);
            return;
        }

        if (t >= 2.70834f && t < 3.04167f)
            DrawClipAt(g, clips["vfx_area_front"], t - 2.70834f, fighter.Pos.X, fighter.Pos.Y,
                fighter.Facing, fighter.Def.RenderScale, 2f);
        if (t >= 2.70834f && t < 3.83333f)
            DrawClipAt(g, clips["vfx_area_explosion"], t - 2.70834f, fighter.Pos.X, fighter.Pos.Y,
                fighter.Facing, fighter.Def.RenderScale, 2f);
        if (t >= 3.375f && t < 4.20834f)
            DrawClipAt(g, clips["vfx_area_smoke"], t - 3.375f, fighter.Pos.X, fighter.Pos.Y,
                fighter.Facing, fighter.Def.RenderScale, 2f);
    }

    private void DrawTaxDeath(Graphics g)
    {
        if (!_taxDeathActive || _taxDeathAge > 4.54167f) return;
        var clips = SpriteLibrary.Get("taxman");
        float t = _taxDeathAge;
        float scale = Game.WorldScale;

        if (t >= 1.83333f && t < 3.04167f)
            DrawClipAt(g, clips["vfx_death_back"], t - 1.83333f, _taxDeathPos.X, _taxDeathPos.Y,
                _taxDeathFacing, scale, 2f);
        if (t >= 2.625f && t < 3f)
            DrawClipAt(g, clips["vfx_death_ground"], t - 2.625f, _taxDeathPos.X, _taxDeathPos.Y,
                _taxDeathFacing, scale, 2f);

        if (t < 3.33333f)
            DrawClipAt(g, clips["death"], t, _taxDeathPos.X, _taxDeathPos.Y,
                _taxDeathFacing, scale);

        if (t >= 2.95833f && t < 3.29167f)
            DrawClipAt(g, clips["vfx_death_front"], t - 2.95833f, _taxDeathPos.X, _taxDeathPos.Y,
                _taxDeathFacing, scale, 2f);
        if (t >= 3f && t < 4.125f)
            DrawClipAt(g, clips["vfx_death_explosion"], t - 3f, _taxDeathPos.X, _taxDeathPos.Y,
                _taxDeathFacing, scale, 2f);
        if (t >= 3.33333f && t < 4.54167f)
        {
            DrawClipAt(g, clips["vfx_death_smoke_v"], t - 3.33333f, _taxDeathPos.X, _taxDeathPos.Y,
                _taxDeathFacing, scale, 2f);
            DrawClipAt(g, clips["vfx_death_smoke_h"], t - 3.33333f, _taxDeathPos.X, _taxDeathPos.Y,
                _taxDeathFacing, scale, 2f);
            DrawClipAt(g, clips["vfx_coins"], t - 3.33333f, _taxDeathPos.X, _taxDeathPos.Y,
                _taxDeathFacing, scale);
        }
    }

    private static void DrawImageAlpha(Graphics g, Bitmap bmp, RectangleF dest, float a)
    {
        var cm = new ColorMatrix { Matrix33 = a };
        using var ia = new ImageAttributes();
        ia.SetColorMatrix(cm);
        g.DrawImage(bmp, Rect(dest), 0, 0, bmp.Width, bmp.Height, GraphicsUnit.Pixel, ia);
    }

    private static void DrawImageWhite(Graphics g, Bitmap bmp, RectangleF dest, float a)
    {
        // keep source alpha, force RGB to white, scale by a
        var cm = new ColorMatrix(new float[][]
        {
            new float[] {0,0,0,0,0},
            new float[] {0,0,0,0,0},
            new float[] {0,0,0,0,0},
            new float[] {0,0,0,a,0},
            new float[] {1,1,1,0,1},
        });
        using var ia = new ImageAttributes();
        ia.SetColorMatrix(cm);
        g.DrawImage(bmp, Rect(dest), 0, 0, bmp.Width, bmp.Height, GraphicsUnit.Pixel, ia);
    }

    private static Rectangle Rect(RectangleF r) =>
        new((int)MathF.Round(r.X), (int)MathF.Round(r.Y), (int)MathF.Round(r.Width), (int)MathF.Round(r.Height));

    private static void DrawShadow(Graphics g, Fighter f)
    {
        float shrink = 1f / (1f + f.SkinY * 0.004f);
        float rx = (f.Def.Radius + 8f) * shrink;
        float ry = rx * 0.36f;
        int a = (int)(90 * (f.Dead ? Math.Clamp(f.DeathTimer, 0f, 1f) : 1f) * shrink);
        using var b = new SolidBrush(Color.FromArgb(Math.Clamp(a, 0, 120), 0, 0, 0));
        g.FillEllipse(b, f.Pos.X - rx, f.Pos.Y - ry, rx * 2, ry * 2);
    }

    private void DrawHud(Graphics g)
    {
        const float scale = Game.WorldScale;
        DrawLifeBar(g, 50f * scale, 30f * scale, scale,
            _hudPlayerBase, _hudPlayerUnder, _hudPlayerProgress, _hudHeart, _profileChad,
            _game.Player.Health / (float)_game.Player.MaxHealth, enemy: false);

        Fighter? target = null;
        float best = float.MaxValue;
        foreach (Fighter fighter in _game.Fighters)
        {
            if (fighter.Faction != Faction.Enemy || !fighter.Alive) continue;
            float dx = MathF.Abs(fighter.Pos.X - _game.Player.Pos.X);
            if (dx < best) { best = dx; target = fighter; }
        }
        if (target != null)
        {
            Image? profile = target.Def.SpriteSet == "taxman" ? _profileTaxman : _profileSarge;
            DrawLifeBar(g, 50f * scale, 158f * scale, scale,
                _hudEnemyBase, _hudEnemyUnder, _hudEnemyProgress, _hudEnemyHeart, profile,
                target.Health / (float)target.MaxHealth, enemy: true);
        }
    }

    private static void DrawLifeBar(Graphics g, float x, float y, float scale,
        Image? frame, Image? under, Image? progress, Image? heart, Image? profile,
        float percentage, bool enemy)
    {
        percentage = Math.Clamp(percentage, 0f, 1f);
        float profileLeft = enemy ? 5f : 8f;
        float profileTop = enemy ? 3f : 5f;
        float profileWidth = enemy ? 130f : 171f;
        float profileHeight = enemy ? 90f : 118f;
        float heartLeft = enemy ? 147f : 197f;
        float heartTop = enemy ? 7f : 10f;
        float progressLeft = enemy ? 186f : 247f;

        // Godot draws the TextureRect base first, then its Profile, Heart and
        // TextureProgressBar children. Drawing the base last hides all children.
        if (frame != null)
            g.DrawImage(frame, x, y, frame.Width * scale, frame.Height * scale);
        if (profile != null)
            g.DrawImage(profile, x + profileLeft * scale, y + profileTop * scale,
                profileWidth * scale, profileHeight * scale);
        if (under != null)
            g.DrawImage(under, x + progressLeft * scale, y,
                under.Width * scale, under.Height * scale);
        if (progress != null && percentage > 0f)
        {
            int sourceWidth = Math.Max(1, (int)MathF.Round(progress.Width * percentage));
            var destination = new RectangleF(x + progressLeft * scale, y,
                sourceWidth * scale, progress.Height * scale);
            g.DrawImage(progress, destination, new Rectangle(0, 0, sourceWidth, progress.Height),
                GraphicsUnit.Pixel);
        }
        if (heart != null)
            g.DrawImage(heart, x + heartLeft * scale, y + heartTop * scale,
                heart.Width * scale, heart.Height * scale);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _timer.Dispose();
            _buffer.Dispose();
            _stageRenderer.Dispose();
            _fEndTitle.Dispose();
            _fEndHint.Dispose();
            foreach (Image? image in new[] {
                _hudPlayerBase, _hudPlayerUnder, _hudPlayerProgress, _hudHeart,
                _hudEnemyBase, _hudEnemyUnder, _hudEnemyProgress, _hudEnemyHeart,
                _profileChad, _profileSarge, _profileTaxman })
                image?.Dispose();
        }
        base.Dispose(disposing);
    }
}
