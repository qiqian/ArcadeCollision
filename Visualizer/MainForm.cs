using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ArcCollision.Visualizer;

/// <summary>
/// Interactive playground for the ArcCollision reference library. Pick a scene
/// from the dropdown and drag the coloured handles to explore how the collision
/// routines respond in real time.
/// </summary>
public sealed class MainForm : Form
{
    private readonly ComboBox _sceneBox = new();
    private readonly Label _info = new();
    private readonly CheckBox _showContacts = new();
    private readonly CanvasControl _canvas;

    public MainForm()
    {
        Text = "ArcCollision Visualizer";
        BackColor = Color.FromArgb(24, 26, 32);
        ClientSize = new Size(1100, 720);
        MinimumSize = new Size(760, 520);
        DoubleBuffered = true;

        _canvas = new CanvasControl { Dock = DockStyle.Fill };

        var side = new Panel
        {
            Dock = DockStyle.Left,
            Width = 260,
            BackColor = Color.FromArgb(32, 35, 43),
            Padding = new Padding(16),
        };

        var title = new Label
        {
            Text = "ArcCollision",
            ForeColor = Color.FromArgb(120, 220, 255),
            Font = new Font("Segoe UI Semibold", 16f, FontStyle.Bold),
            AutoSize = true,
            Location = new Point(16, 16),
        };
        var subtitle = new Label
        {
            Text = "Interactive collision explorer",
            ForeColor = Color.Gainsboro,
            Font = new Font("Segoe UI", 9f),
            AutoSize = true,
            Location = new Point(16, 46),
        };

        var sceneLabel = new Label
        {
            Text = "Scene",
            ForeColor = Color.Silver,
            AutoSize = true,
            Location = new Point(16, 84),
        };
        _sceneBox.DropDownStyle = ComboBoxStyle.DropDownList;
        _sceneBox.Location = new Point(16, 104);
        _sceneBox.Width = 220;
        _sceneBox.Items.AddRange(new object[]
        {
            "Circle vs Circle",
            "Circle vs AABB",
            "AABB vs AABB",
            "Capsule vs Capsule",
            "Capsule vs AABB",
            "Swept: Circle vs Circle",
            "Swept: Circle vs AABB",
        });
        _sceneBox.SelectedIndex = 0;
        _sceneBox.SelectedIndexChanged += (_, _) =>
        {
            _canvas.SetScene((SceneKind)_sceneBox.SelectedIndex);
            _canvas.Invalidate();
        };

        _showContacts.Text = "Show contact & normal";
        _showContacts.ForeColor = Color.Silver;
        _showContacts.Checked = true;
        _showContacts.AutoSize = true;
        _showContacts.Location = new Point(16, 142);
        _showContacts.CheckedChanged += (_, _) =>
        {
            _canvas.ShowContacts = _showContacts.Checked;
            _canvas.Invalidate();
        };

        _info.ForeColor = Color.FromArgb(200, 220, 235);
        _info.Font = new Font("Consolas", 9.5f);
        _info.AutoSize = false;
        _info.Location = new Point(16, 180);
        _info.Size = new Size(224, 380);

        var help = new Label
        {
            Text = "Drag the round handles.\n" +
                   "Big dots move a whole shape;\n" +
                   "small dots resize it.\n" +
                   "Green dot sets swept motion.",
            ForeColor = Color.Gray,
            Font = new Font("Segoe UI", 8.5f),
            AutoSize = false,
            Size = new Size(224, 90),
            Dock = DockStyle.Bottom,
        };

        side.Controls.Add(_info);
        side.Controls.Add(_showContacts);
        side.Controls.Add(_sceneBox);
        side.Controls.Add(sceneLabel);
        side.Controls.Add(subtitle);
        side.Controls.Add(title);
        side.Controls.Add(help);

        _canvas.InfoChanged += text => _info.Text = text;
        _canvas.SetScene(SceneKind.CircleCircle);

        Controls.Add(_canvas);
        Controls.Add(side);
    }
}

internal enum SceneKind
{
    CircleCircle = 0,
    CircleAabb,
    AabbAabb,
    CapsuleCapsule,
    CapsuleAabb,
    SweptCircleCircle,
    SweptCircleAabb,
}

/// <summary>A draggable control point.</summary>
internal sealed class Handle
{
    public required string Name;
    public Vec2 Pos;
    public bool IsCenter;
    public readonly List<string> Children = new();
}

internal sealed class CanvasControl : Control
{
    private readonly Dictionary<string, Handle> _handles = new();
    private SceneKind _scene;
    private string? _dragging;
    private Vec2 _dragOffset;

    public bool ShowContacts = true;
    public event Action<string>? InfoChanged;

    public CanvasControl()
    {
        DoubleBuffered = true;
        SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint, true);
        BackColor = Color.FromArgb(18, 20, 25);
    }

    // ------------------------------------------------------------- scene setup

    public void SetScene(SceneKind scene)
    {
        _scene = scene;
        _handles.Clear();
        var c = new Vec2(Math.Max(ClientSize.Width, 600) * 0.55f, Math.Max(ClientSize.Height, 400) * 0.5f);

        switch (scene)
        {
            case SceneKind.CircleCircle:
                AddCircle("A", c + new Vec2(-120, 0), 70);
                AddCircle("B", c + new Vec2(120, 0), 90);
                break;
            case SceneKind.CircleAabb:
                AddCircle("A", c + new Vec2(-130, 0), 70);
                AddBox("B", c + new Vec2(110, 0), new Vec2(90, 70));
                break;
            case SceneKind.AabbAabb:
                AddBox("A", c + new Vec2(-120, 0), new Vec2(80, 90));
                AddBox("B", c + new Vec2(110, 10), new Vec2(90, 70));
                break;
            case SceneKind.CapsuleCapsule:
                AddCapsule("A", c + new Vec2(-140, -60), c + new Vec2(-60, 80), 34);
                AddCapsule("B", c + new Vec2(120, -70), c + new Vec2(60, 70), 40);
                break;
            case SceneKind.CapsuleAabb:
                AddCapsule("A", c + new Vec2(-150, -60), c + new Vec2(-40, 70), 34);
                AddBox("B", c + new Vec2(120, 0), new Vec2(80, 90));
                break;
            case SceneKind.SweptCircleCircle:
                AddCircle("A", c + new Vec2(-230, 0), 40);
                AddCircle("B", c + new Vec2(80, 0), 80);
                AddMotion("A", c + new Vec2(60, 0));
                break;
            case SceneKind.SweptCircleAabb:
                AddCircle("A", c + new Vec2(-230, 0), 36);
                AddBox("B", c + new Vec2(90, 0), new Vec2(90, 80));
                AddMotion("A", c + new Vec2(80, 0));
                break;
        }
    }

    private void AddCircle(string key, Vec2 center, float radius)
    {
        var ctr = new Handle { Name = key + ".c", Pos = center, IsCenter = true };
        var edge = new Handle { Name = key + ".r", Pos = center + new Vec2(radius, 0) };
        ctr.Children.Add(edge.Name);
        _handles[ctr.Name] = ctr;
        _handles[edge.Name] = edge;
    }

    private void AddBox(string key, Vec2 center, Vec2 half)
    {
        var ctr = new Handle { Name = key + ".c", Pos = center, IsCenter = true };
        var corner = new Handle { Name = key + ".e", Pos = center + half };
        ctr.Children.Add(corner.Name);
        _handles[ctr.Name] = ctr;
        _handles[corner.Name] = corner;
    }

    private void AddCapsule(string key, Vec2 a, Vec2 b, float radius)
    {
        var ha = new Handle { Name = key + ".a", Pos = a, IsCenter = true };
        var hb = new Handle { Name = key + ".b", Pos = b, IsCenter = true };
        var perp = (b - a).Perp.Normalized(Vec2.UnitY);
        var hr = new Handle { Name = key + ".r", Pos = a + perp * radius };
        ha.Children.Add(hr.Name);
        _handles[ha.Name] = ha;
        _handles[hb.Name] = hb;
        _handles[hr.Name] = hr;
    }

    private void AddMotion(string key, Vec2 end)
    {
        _handles[key + ".m"] = new Handle { Name = key + ".m", Pos = end };
    }

    // ------------------------------------------------------------ shape access

    private Vec2 P(string name) => _handles[name].Pos;
    private Circle GetCircle(string key) => new(P(key + ".c"), MathF.Max(6f, P(key + ".c").Distance(P(key + ".r"))));
    private Aabb GetBox(string key)
    {
        Vec2 c = P(key + ".c");
        Vec2 e = P(key + ".e");
        return new Aabb(c, new Vec2(MathF.Max(6f, MathF.Abs(e.X - c.X)), MathF.Max(6f, MathF.Abs(e.Y - c.Y))));
    }
    private Capsule GetCapsule(string key) => new(P(key + ".a"), P(key + ".b"), MathF.Max(6f, P(key + ".a").Distance(P(key + ".r"))));

    // -------------------------------------------------------------- input

    protected override void OnMouseDown(MouseEventArgs e)
    {
        var mouse = new Vec2(e.X, e.Y);
        Handle? best = null;
        float bestDist = 14f; // pick radius
        foreach (var h in _handles.Values)
        {
            float d = h.Pos.Distance(mouse);
            if (d <= bestDist)
            {
                bestDist = d;
                best = h;
            }
        }
        if (best != null)
        {
            _dragging = best.Name;
            _dragOffset = best.Pos - mouse;
        }
        base.OnMouseDown(e);
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        if (_dragging != null && _handles.TryGetValue(_dragging, out var h))
        {
            var newPos = new Vec2(e.X, e.Y) + _dragOffset;
            var delta = newPos - h.Pos;
            h.Pos = newPos;
            foreach (var childName in h.Children)
                if (_handles.TryGetValue(childName, out var child))
                    child.Pos += delta;
            Invalidate();
        }
        base.OnMouseMove(e);
    }

    protected override void OnMouseUp(MouseEventArgs e)
    {
        _dragging = null;
        base.OnMouseUp(e);
    }

    protected override void OnResize(EventArgs e)
    {
        base.OnResize(e);
        Invalidate();
    }

    // -------------------------------------------------------------- rendering

    private static readonly Color ColA = Color.FromArgb(90, 200, 255);
    private static readonly Color ColB = Color.FromArgb(255, 170, 90);
    private static readonly Color ColHit = Color.FromArgb(255, 90, 110);
    private static readonly Color ColNormal = Color.FromArgb(255, 230, 90);
    private static readonly Color ColMotion = Color.FromArgb(120, 235, 140);

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.Clear(BackColor);
        DrawGrid(g);

        switch (_scene)
        {
            case SceneKind.CircleCircle: DrawDiscrete(g, GetCircle("A"), GetCircle("B")); break;
            case SceneKind.CircleAabb: DrawDiscrete(g, GetCircle("A"), GetBox("B")); break;
            case SceneKind.AabbAabb: DrawDiscrete(g, GetBox("A"), GetBox("B")); break;
            case SceneKind.CapsuleCapsule: DrawDiscrete(g, GetCapsule("A"), GetCapsule("B")); break;
            case SceneKind.CapsuleAabb: DrawDiscrete(g, GetCapsule("A"), GetBox("B")); break;
            case SceneKind.SweptCircleCircle: DrawSwept(g, isBox: false); break;
            case SceneKind.SweptCircleAabb: DrawSwept(g, isBox: true); break;
        }

        DrawHandles(g);
    }

    private void DrawGrid(Graphics g)
    {
        using var pen = new Pen(Color.FromArgb(32, 255, 255, 255));
        const int step = 40;
        for (int x = 0; x < Width; x += step) g.DrawLine(pen, x, 0, x, Height);
        for (int y = 0; y < Height; y += step) g.DrawLine(pen, 0, y, Width, y);
    }

    // ---- discrete (dispatch by dynamic type to reuse one code path) ----

    private void DrawDiscrete(Graphics g, object a, object b)
    {
        Manifold m = Narrow(a, b);
        DrawShape(g, a, m.Colliding ? ColHit : ColA);
        DrawShape(g, b, m.Colliding ? ColHit : ColB);

        if (m.Colliding && ShowContacts)
        {
            DrawArrow(g, m.Contact, m.Contact + m.Normal * 60f, ColNormal, 2.4f);
            FillDot(g, m.Contact, 5f, Color.White);
        }

        InfoChanged?.Invoke(
            $"Scene: {_scene}\n\n" +
            (m.Colliding
                ? $"COLLIDING\n\ndepth  : {m.Depth,7:0.00}\nnormal : ({m.Normal.X,5:0.00}, {m.Normal.Y,5:0.00})\ncontact: ({m.Contact.X,5:0.0}, {m.Contact.Y,5:0.0})"
                : "no collision"));
    }

    private static Manifold Narrow(object a, object b) => (a, b) switch
    {
        (Circle ca, Circle cb) => Collide.CircleVsCircle(ca, cb),
        (Circle ca, Aabb bb) => Collide.CircleVsAabb(ca, bb),
        (Aabb ba, Aabb bb) => Collide.AabbVsAabb(ba, bb),
        (Capsule pa, Capsule pb) => Collide.CapsuleVsCapsule(pa, pb),
        (Capsule pa, Aabb bb) => Collide.CapsuleVsAabb(pa, bb),
        _ => Manifold.None,
    };

    private void DrawSwept(Graphics g, bool isBox)
    {
        Circle mover = GetCircle("A");
        Vec2 motion = P("A.m") - mover.Center;

        SweepHit hit = isBox
            ? Sweep.MovingCircleVsAabb(mover, motion, GetBox("B"))
            : Sweep.MovingCircleVsCircle(mover, motion, GetCircle("B"));

        // target
        if (isBox) DrawShape(g, GetBox("B"), ColB);
        else DrawShape(g, GetCircle("B"), ColB);

        // motion path
        DrawArrow(g, mover.Center, P("A.m"), ColMotion, 2f);

        // start (solid) + end (faint) mover
        DrawCircleOutline(g, mover.Center, mover.Radius, ColA, 2f);
        DrawCircleOutline(g, mover.Center + motion, mover.Radius, Color.FromArgb(70, ColA), 1.5f);

        if (hit.Hit)
        {
            Vec2 stop = mover.Center + motion * hit.Time;
            DrawCircleOutline(g, stop, mover.Radius, ColHit, 2.4f);
            if (ShowContacts)
            {
                FillDot(g, hit.Point, 5f, Color.White);
                DrawArrow(g, hit.Point, hit.Point + hit.Normal * 50f, ColNormal, 2.2f);
            }
        }

        InfoChanged?.Invoke(
            $"Scene: {_scene}\n\n" +
            (hit.Hit
                ? $"IMPACT\n\ntime   : {hit.Time,7:0.000}\nnormal : ({hit.Normal.X,5:0.00}, {hit.Normal.Y,5:0.00})\npoint  : ({hit.Point.X,5:0.0}, {hit.Point.Y,5:0.0})"
                : "clear path (no impact)"));
    }

    // ---- shape drawing dispatch ----

    private void DrawShape(Graphics g, object shape, Color color)
    {
        switch (shape)
        {
            case Circle c: DrawCircleFilled(g, c.Center, c.Radius, color); break;
            case Aabb b: DrawBox(g, b, color); break;
            case Capsule cap: DrawCapsuleShape(g, cap, color); break;
        }
    }

    private static void DrawCircleFilled(Graphics g, Vec2 c, float r, Color color)
    {
        using var fill = new SolidBrush(Color.FromArgb(60, color));
        using var pen = new Pen(color, 2f);
        g.FillEllipse(fill, c.X - r, c.Y - r, r * 2, r * 2);
        g.DrawEllipse(pen, c.X - r, c.Y - r, r * 2, r * 2);
    }

    private static void DrawCircleOutline(Graphics g, Vec2 c, float r, Color color, float w)
    {
        using var pen = new Pen(color, w);
        g.DrawEllipse(pen, c.X - r, c.Y - r, r * 2, r * 2);
    }

    private static void DrawBox(Graphics g, Aabb b, Color color)
    {
        var min = b.Min;
        using var fill = new SolidBrush(Color.FromArgb(60, color));
        using var pen = new Pen(color, 2f);
        var rect = new RectangleF(min.X, min.Y, b.HalfExtents.X * 2, b.HalfExtents.Y * 2);
        g.FillRectangle(fill, rect);
        g.DrawRectangle(pen, rect.X, rect.Y, rect.Width, rect.Height);
    }

    private static void DrawCapsuleShape(Graphics g, Capsule cap, Color color)
    {
        using var fill = new SolidBrush(Color.FromArgb(60, color));
        using var pen = new Pen(color, 2f);
        Vec2 dir = (cap.B - cap.A).Normalized(Vec2.UnitX);
        Vec2 n = dir.Perp * cap.Radius;

        var path = new GraphicsPath();
        // one side, cap at B, other side, cap at A
        Vec2 a1 = cap.A + n, a2 = cap.A - n, b1 = cap.B + n, b2 = cap.B - n;
        path.AddLine(a1.X, a1.Y, b1.X, b1.Y);
        AddArc(path, cap.B, cap.Radius, Angle(n), Angle(-n));
        path.AddLine(b2.X, b2.Y, a2.X, a2.Y);
        AddArc(path, cap.A, cap.Radius, Angle(-n), Angle(n));
        path.CloseFigure();

        g.FillPath(fill, path);
        g.DrawPath(pen, path);
        path.Dispose();
    }

    private static float Angle(Vec2 v) => MathF.Atan2(v.Y, v.X) * 180f / MathF.PI;

    private static void AddArc(GraphicsPath path, Vec2 center, float r, float startDeg, float endDeg)
    {
        float sweep = endDeg - startDeg;
        while (sweep <= 0) sweep += 360f;
        path.AddArc(center.X - r, center.Y - r, r * 2, r * 2, startDeg, sweep);
    }

    private void DrawHandles(Graphics g)
    {
        foreach (var h in _handles.Values)
        {
            bool motion = h.Name.EndsWith(".m");
            float r = h.IsCenter ? 6.5f : 5f;
            Color col = motion ? ColMotion : (h.IsCenter ? Color.White : Color.FromArgb(200, 200, 210));
            FillDot(g, h.Pos, r, col);
            using var ring = new Pen(Color.FromArgb(120, 0, 0, 0), 1.5f);
            g.DrawEllipse(ring, h.Pos.X - r, h.Pos.Y - r, r * 2, r * 2);
        }
    }

    private static void FillDot(Graphics g, Vec2 p, float r, Color color)
    {
        using var b = new SolidBrush(color);
        g.FillEllipse(b, p.X - r, p.Y - r, r * 2, r * 2);
    }

    private static void DrawArrow(Graphics g, Vec2 from, Vec2 to, Color color, float width)
    {
        using var pen = new Pen(color, width);
        g.DrawLine(pen, from.X, from.Y, to.X, to.Y);
        Vec2 dir = (to - from).Normalized(Vec2.UnitX);
        Vec2 left = dir.Perp;
        float head = 9f;
        Vec2 p1 = to - dir * head + left * (head * 0.55f);
        Vec2 p2 = to - dir * head - left * (head * 0.55f);
        using var brush = new SolidBrush(color);
        g.FillPolygon(brush, new[]
        {
            new PointF(to.X, to.Y),
            new PointF(p1.X, p1.Y),
            new PointF(p2.X, p2.Y),
        });
    }
}
