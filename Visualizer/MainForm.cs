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
    private readonly ShapeLegendGlyph _legendStart = new();
    private readonly ShapeLegendGlyph _legendTarget = new();
    private readonly ShapeLegendGlyph _legendEnd = new();
    private readonly ShapeLegendGlyph _legendImpact = new();

    public MainForm()
    {
        Text = "ArcCollision Visualizer";
        BackColor = Color.FromArgb(24, 26, 32);
        ClientSize = new Size(1100, 720);
        MinimumSize = new Size(820, 640);
        DoubleBuffered = true;

        _canvas = new CanvasControl { Dock = DockStyle.Fill };

        var side = new Panel
        {
            Dock = DockStyle.Left,
            Width = 340,
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
        _sceneBox.Width = 300;
        _sceneBox.DropDownWidth = 320;
        _sceneBox.Items.AddRange(new object[]
        {
            "Circle vs Circle",
            "Circle vs AABB",
            "Circle vs Capsule",
            "Circle vs OBB",
            "Circle vs Polygon",
            "AABB vs AABB",
            "Capsule vs AABB",
            "AABB vs OBB",
            "AABB vs Polygon",
            "Capsule vs Capsule",
            "Capsule vs OBB",
            "Capsule vs Polygon",
            "OBB vs OBB",
            "OBB vs Polygon",
            "Polygon vs Polygon",
            "Swept: Circle vs Circle",
            "Swept: Circle vs AABB",
            "Ray vs Circle",
            "Ray vs AABB",
        });
        foreach (ShapeKind mover in Enum.GetValues<ShapeKind>())
        foreach (ShapeKind target in Enum.GetValues<ShapeKind>())
            _sceneBox.Items.Add($"Swept: {mover} vs {target}");
        _sceneBox.SelectedIndex = 0;
        _sceneBox.SelectedIndexChanged += (_, _) =>
        {
            int index = _sceneBox.SelectedIndex;
            int fixedScenes = (int)SceneKind.RayAabb + 1;
            if (index < fixedScenes)
            {
                _canvas.SetScene((SceneKind)index);
            }
            else
            {
                int pair = index - fixedScenes;
                int shapeCount = Enum.GetValues<ShapeKind>().Length;
                _canvas.SetSweptScene(
                    (ShapeKind)(pair / shapeCount),
                    (ShapeKind)(pair % shapeCount));
            }
            UpdateLegend(index);
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
        _info.Font = new Font("Consolas", 9f);
        _info.AutoSize = false;
        _info.Location = new Point(16, 288);
        _info.Size = new Size(304, 300);

        var legend = new Panel
        {
            Location = new Point(16, 180),
            Size = new Size(304, 96),
            BackColor = Color.Transparent,
        };
        AddLegendRow(legend, 0, _legendStart,
            Color.FromArgb(90, 200, 255), "A 起点 / start");
        AddLegendRow(legend, 22, _legendTarget,
            Color.FromArgb(255, 170, 90), "B 目标 / target");
        AddLegendRow(legend, 44, _legendEnd,
            Color.FromArgb(55, 115, 145), "A 终点虚影 / end ghost");
        AddLegendRow(legend, 66, _legendImpact,
            Color.FromArgb(255, 90, 110), "A 碰撞位置 / at TOI");

        var help = new Label
        {
            Text = "Drag the round handles.\n" +
                   "Big dots move a whole shape;\n" +
                   "small dots resize/rotate it.\n" +
                   "Green dot sets swept motion.",
            ForeColor = Color.Gray,
            Font = new Font("Segoe UI", 8.5f),
            AutoSize = false,
            Size = new Size(304, 90),
            Dock = DockStyle.Bottom,
        };

        side.Controls.Add(_info);
        side.Controls.Add(legend);
        side.Controls.Add(_showContacts);
        side.Controls.Add(_sceneBox);
        side.Controls.Add(sceneLabel);
        side.Controls.Add(subtitle);
        side.Controls.Add(title);
        side.Controls.Add(help);
        side.Resize += (_, _) =>
        {
            _info.Width = Math.Max(120, side.ClientSize.Width - 32);
            _info.Height = Math.Max(170,
                side.ClientSize.Height - _info.Top - help.Height - 20);
        };

        _canvas.InfoChanged += text => _info.Text = text;
        _canvas.SetScene(SceneKind.CircleCircle);
        UpdateLegend(0);

        Controls.Add(_canvas);
        Controls.Add(side);
    }

    private static void AddLegendRow(
        Control parent, int y, ShapeLegendGlyph glyph, Color color, string text)
    {
        glyph.Location = new Point(0, y);
        glyph.Size = new Size(20, 20);
        glyph.GlyphColor = color;
        var label = new Label
        {
            Location = new Point(28, y),
            Text = text,
            ForeColor = Color.Gainsboro,
            Font = new Font("Segoe UI", 9f),
            AutoSize = true,
            MaximumSize = new Size(278, 0),
        };
        parent.Controls.Add(glyph);
        parent.Controls.Add(label);
    }

    private void UpdateLegend(int selectedIndex)
    {
        int fixedScenes = (int)SceneKind.RayAabb + 1;
        ShapeKind mover;
        ShapeKind target;
        bool ray = false;
        if (selectedIndex >= fixedScenes)
        {
            int shapeCount = Enum.GetValues<ShapeKind>().Length;
            int pair = selectedIndex - fixedScenes;
            mover = (ShapeKind)(pair / shapeCount);
            target = (ShapeKind)(pair % shapeCount);
        }
        else
        {
            (mover, target, ray) = SceneShapes((SceneKind)selectedIndex);
        }

        _legendStart.SetShape(mover, ray);
        _legendEnd.SetShape(mover, ray);
        _legendImpact.SetShape(mover, ray);
        _legendTarget.SetShape(target, false);
    }

    private static (ShapeKind Mover, ShapeKind Target, bool Ray) SceneShapes(
        SceneKind scene) => scene switch
    {
        SceneKind.CircleCircle or SceneKind.SweptCircleCircle =>
            (ShapeKind.Circle, ShapeKind.Circle, false),
        SceneKind.CircleAabb or SceneKind.SweptCircleAabb =>
            (ShapeKind.Circle, ShapeKind.Aabb, false),
        SceneKind.CircleCapsule => (ShapeKind.Circle, ShapeKind.Capsule, false),
        SceneKind.CircleObb => (ShapeKind.Circle, ShapeKind.Obb, false),
        SceneKind.CirclePolygon => (ShapeKind.Circle, ShapeKind.Polygon, false),
        SceneKind.AabbAabb => (ShapeKind.Aabb, ShapeKind.Aabb, false),
        SceneKind.CapsuleAabb => (ShapeKind.Capsule, ShapeKind.Aabb, false),
        SceneKind.AabbObb => (ShapeKind.Aabb, ShapeKind.Obb, false),
        SceneKind.AabbPolygon => (ShapeKind.Aabb, ShapeKind.Polygon, false),
        SceneKind.CapsuleCapsule => (ShapeKind.Capsule, ShapeKind.Capsule, false),
        SceneKind.CapsuleObb => (ShapeKind.Capsule, ShapeKind.Obb, false),
        SceneKind.CapsulePolygon => (ShapeKind.Capsule, ShapeKind.Polygon, false),
        SceneKind.ObbObb => (ShapeKind.Obb, ShapeKind.Obb, false),
        SceneKind.ObbPolygon => (ShapeKind.Obb, ShapeKind.Polygon, false),
        SceneKind.PolygonPolygon => (ShapeKind.Polygon, ShapeKind.Polygon, false),
        SceneKind.RayCircle => (ShapeKind.Circle, ShapeKind.Circle, true),
        SceneKind.RayAabb => (ShapeKind.Circle, ShapeKind.Aabb, true),
        _ => throw new ArgumentOutOfRangeException(nameof(scene)),
    };
}

internal sealed class ShapeLegendGlyph : Control
{
    private ShapeKind _kind;
    private bool _ray;

    public Color GlyphColor { get; set; } = Color.White;

    public ShapeLegendGlyph()
    {
        SetStyle(ControlStyles.UserPaint | ControlStyles.SupportsTransparentBackColor
            | ControlStyles.OptimizedDoubleBuffer, true);
        BackColor = Color.Transparent;
    }

    public void SetShape(ShapeKind kind, bool ray)
    {
        _kind = kind;
        _ray = ray;
        Invalidate();
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        using var fill = new SolidBrush(Color.FromArgb(90, GlyphColor));
        using var pen = new Pen(GlyphColor, 1.7f);

        if (_ray)
        {
            g.DrawLine(pen, 2, 15, 17, 5);
            using var arrow = new SolidBrush(GlyphColor);
            g.FillPolygon(arrow, new[]
            {
                new PointF(17, 5), new PointF(11, 6), new PointF(15, 11),
            });
            return;
        }

        switch (_kind)
        {
            case ShapeKind.Circle:
                g.FillEllipse(fill, 3, 3, 14, 14);
                g.DrawEllipse(pen, 3, 3, 14, 14);
                break;
            case ShapeKind.Aabb:
                g.FillRectangle(fill, 2, 4, 16, 12);
                g.DrawRectangle(pen, 2, 4, 16, 12);
                break;
            case ShapeKind.Capsule:
                using (var path = new GraphicsPath())
                {
                    path.AddArc(1, 5, 10, 10, 90, 180);
                    path.AddLine(6, 5, 14, 5);
                    path.AddArc(9, 5, 10, 10, 270, 180);
                    path.AddLine(14, 15, 6, 15);
                    path.CloseFigure();
                    g.FillPath(fill, path);
                    g.DrawPath(pen, path);
                }
                break;
            case ShapeKind.Obb:
                PointF[] obb =
                {
                    new(5, 2), new(19, 7), new(15, 18), new(1, 13),
                };
                g.FillPolygon(fill, obb);
                g.DrawPolygon(pen, obb);
                break;
            case ShapeKind.Polygon:
                PointF[] polygon =
                {
                    new(10, 1), new(19, 7), new(15, 18),
                    new(5, 17), new(1, 7),
                };
                g.FillPolygon(fill, polygon);
                g.DrawPolygon(pen, polygon);
                break;
        }
    }
}

internal enum SceneKind
{
    CircleCircle = 0,
    CircleAabb,
    CircleCapsule,
    CircleObb,
    CirclePolygon,
    AabbAabb,
    CapsuleAabb,
    AabbObb,
    AabbPolygon,
    CapsuleCapsule,
    CapsuleObb,
    CapsulePolygon,
    ObbObb,
    ObbPolygon,
    PolygonPolygon,
    SweptCircleCircle,
    SweptCircleAabb,
    RayCircle,
    RayAabb,
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
    private bool _genericSweep;
    private ShapeKind _sweepMoverKind;
    private ShapeKind _sweepTargetKind;
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
        _genericSweep = false;
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
            case SceneKind.CircleCapsule:
                AddCircle("A", c + new Vec2(-130, 0), 70);
                AddCapsule("B", c + new Vec2(40, -80), c + new Vec2(150, 70), 34);
                break;
            case SceneKind.CircleObb:
                AddCircle("A", c + new Vec2(-130, 0), 70);
                AddObb("B", c + new Vec2(110, 0), new Vec2(90, 60), .45f);
                break;
            case SceneKind.CirclePolygon:
                AddCircle("A", c + new Vec2(-130, 0), 70);
                AddPolygon("B", c + new Vec2(100, 0),
                    new Vec2(-90, -55), new Vec2(20, -90), new Vec2(100, -15),
                    new Vec2(65, 80), new Vec2(-70, 70));
                break;
            case SceneKind.AabbAabb:
                AddBox("A", c + new Vec2(-120, 0), new Vec2(80, 90));
                AddBox("B", c + new Vec2(110, 10), new Vec2(90, 70));
                break;
            case SceneKind.CapsuleAabb:
                AddCapsule("A", c + new Vec2(-150, -60), c + new Vec2(-40, 70), 34);
                AddBox("B", c + new Vec2(120, 0), new Vec2(80, 90));
                break;
            case SceneKind.AabbObb:
                AddBox("A", c + new Vec2(-120, 0), new Vec2(80, 90));
                AddObb("B", c + new Vec2(110, 0), new Vec2(95, 55), -.5f);
                break;
            case SceneKind.AabbPolygon:
                AddBox("A", c + new Vec2(-120, 0), new Vec2(80, 90));
                AddPolygon("B", c + new Vec2(110, 0),
                    new Vec2(-85, -65), new Vec2(55, -80), new Vec2(105, 20),
                    new Vec2(20, 90), new Vec2(-95, 45));
                break;
            case SceneKind.CapsuleCapsule:
                AddCapsule("A", c + new Vec2(-140, -60), c + new Vec2(-60, 80), 34);
                AddCapsule("B", c + new Vec2(120, -70), c + new Vec2(60, 70), 40);
                break;
            case SceneKind.CapsuleObb:
                AddCapsule("A", c + new Vec2(-150, -60), c + new Vec2(-40, 70), 34);
                AddObb("B", c + new Vec2(115, 0), new Vec2(90, 60), .6f);
                break;
            case SceneKind.CapsulePolygon:
                AddCapsule("A", c + new Vec2(-150, -60), c + new Vec2(-40, 70), 34);
                AddPolygon("B", c + new Vec2(110, 0),
                    new Vec2(-80, -70), new Vec2(50, -85), new Vec2(100, 5),
                    new Vec2(45, 85), new Vec2(-90, 55));
                break;
            case SceneKind.ObbObb:
                AddObb("A", c + new Vec2(-115, 0), new Vec2(90, 55), .45f);
                AddObb("B", c + new Vec2(115, 0), new Vec2(80, 70), -.55f);
                break;
            case SceneKind.ObbPolygon:
                AddObb("A", c + new Vec2(-115, 0), new Vec2(90, 55), .45f);
                AddPolygon("B", c + new Vec2(110, 0),
                    new Vec2(-90, -60), new Vec2(25, -90), new Vec2(105, -20),
                    new Vec2(65, 80), new Vec2(-75, 70));
                break;
            case SceneKind.PolygonPolygon:
                AddPolygon("A", c + new Vec2(-105, 0),
                    new Vec2(-90, -60), new Vec2(35, -85), new Vec2(95, -5),
                    new Vec2(40, 80), new Vec2(-85, 55));
                AddPolygon("B", c + new Vec2(105, 0),
                    new Vec2(-80, -70), new Vec2(75, -55), new Vec2(95, 55),
                    new Vec2(0, 20), new Vec2(-70, 85));
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
            case SceneKind.RayCircle:
                AddRay("A", c + new Vec2(-250, -80), c + new Vec2(170, 90));
                AddCircle("B", c + new Vec2(70, 0), 80);
                break;
            case SceneKind.RayAabb:
                AddRay("A", c + new Vec2(-250, -80), c + new Vec2(170, 90));
                AddBox("B", c + new Vec2(70, 0), new Vec2(95, 75));
                break;
        }
    }

    public void SetSweptScene(ShapeKind mover, ShapeKind target)
    {
        _genericSweep = true;
        _sweepMoverKind = mover;
        _sweepTargetKind = target;
        _handles.Clear();
        var center = new Vec2(
            Math.Max(ClientSize.Width, 600) * .55f,
            Math.Max(ClientSize.Height, 400) * .5f);
        AddVisualShape("A", mover, center + new Vec2(-190, 0));
        AddVisualShape("B", target, center + new Vec2(95, 0));
        AddMotion("A", center + new Vec2(250, 0));
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
        var hc = new Handle
        {
            Name = key + ".c",
            Pos = (a + b) * .5f,
            IsCenter = true,
        };
        var ha = new Handle { Name = key + ".a", Pos = a, IsCenter = true };
        var hb = new Handle { Name = key + ".b", Pos = b, IsCenter = true };
        var perp = (b - a).Perp.Normalized(Vec2.UnitY);
        var hr = new Handle { Name = key + ".r", Pos = a + perp * radius };
        ha.Children.Add(hr.Name);
        hc.Children.Add(ha.Name);
        hc.Children.Add(hb.Name);
        hc.Children.Add(hr.Name);
        _handles[hc.Name] = hc;
        _handles[ha.Name] = ha;
        _handles[hb.Name] = hb;
        _handles[hr.Name] = hr;
    }

    private void AddObb(string key, Vec2 center, Vec2 half, float rotation)
    {
        Vec2 axisX = new(MathF.Cos(rotation), MathF.Sin(rotation));
        Vec2 axisY = axisX.Perp;
        var ctr = new Handle { Name = key + ".c", Pos = center, IsCenter = true };
        var hx = new Handle { Name = key + ".x", Pos = center + axisX * half.X };
        var hy = new Handle { Name = key + ".y", Pos = center + axisY * half.Y };
        ctr.Children.Add(hx.Name);
        ctr.Children.Add(hy.Name);
        _handles[ctr.Name] = ctr;
        _handles[hx.Name] = hx;
        _handles[hy.Name] = hy;
    }

    private void AddPolygon(string key, Vec2 center, params Vec2[] localVertices)
    {
        var ctr = new Handle { Name = key + ".c", Pos = center, IsCenter = true };
        _handles[ctr.Name] = ctr;
        for (int i = 0; i < localVertices.Length; i++)
        {
            var vertex = new Handle { Name = $"{key}.p{i}", Pos = center + localVertices[i] };
            ctr.Children.Add(vertex.Name);
            _handles[vertex.Name] = vertex;
        }
    }

    private void AddMotion(string key, Vec2 end)
    {
        _handles[key + ".m"] = new Handle { Name = key + ".m", Pos = end };
    }

    private void AddRay(string key, Vec2 origin, Vec2 end)
    {
        var originHandle = new Handle
        {
            Name = key + ".o",
            Pos = origin,
            IsCenter = true,
        };
        originHandle.Children.Add(key + ".m");
        _handles[originHandle.Name] = originHandle;
        AddMotion(key, end);
    }

    private void AddVisualShape(string key, ShapeKind kind, Vec2 center)
    {
        switch (kind)
        {
            case ShapeKind.Circle:
                AddCircle(key, center, 48);
                break;
            case ShapeKind.Aabb:
                AddBox(key, center, new Vec2(62, 44));
                break;
            case ShapeKind.Capsule:
                AddCapsule(key, center + new Vec2(-48, -22),
                    center + new Vec2(48, 22), 25);
                break;
            case ShapeKind.Obb:
                AddObb(key, center, new Vec2(64, 40), .4f);
                break;
            case ShapeKind.Polygon:
                AddPolygon(key, center,
                    new Vec2(-62, -42), new Vec2(20, -58), new Vec2(68, -5),
                    new Vec2(38, 52), new Vec2(-55, 45));
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(kind));
        }
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
    private Obb GetObb(string key)
    {
        Vec2 center = P(key + ".c");
        Vec2 x = P(key + ".x") - center;
        float halfX = MathF.Max(6f, x.Length);
        Vec2 axisX = x.Normalized(Vec2.UnitX);
        float halfY = MathF.Max(6f, MathF.Abs((P(key + ".y") - center).Dot(axisX.Perp)));
        return new Obb(center, new Vec2(halfX, halfY), MathF.Atan2(axisX.Y, axisX.X));
    }

    private Polygon GetPolygon(string key)
    {
        var vertices = new List<Vec2>();
        for (int i = 0; _handles.TryGetValue($"{key}.p{i}", out Handle? handle); i++)
            vertices.Add(handle.Pos);
        return new Polygon(vertices.ToArray());
    }

    private object GetVisualShape(string key, ShapeKind kind) => kind switch
    {
        ShapeKind.Circle => GetCircle(key),
        ShapeKind.Aabb => GetBox(key),
        ShapeKind.Capsule => GetCapsule(key),
        ShapeKind.Obb => GetObb(key),
        ShapeKind.Polygon => GetPolygon(key),
        _ => throw new ArgumentOutOfRangeException(nameof(kind)),
    };

    private static Vec2 VisualCenter(object shape) => shape switch
    {
        Circle value => value.Center,
        Aabb value => value.Center,
        Capsule value => (value.A + value.B) * .5f,
        Obb value => value.Center,
        Polygon value => value.Bounds.Center,
        _ => throw new ArgumentOutOfRangeException(nameof(shape)),
    };

    private static object MoveVisualShape(object shape, Vec2 delta) => shape switch
    {
        Circle value => value.Moved(delta),
        Aabb value => value.Moved(delta),
        Capsule value => value.Moved(delta),
        Obb value => value.Moved(delta),
        Polygon value => value.Moved(delta),
        _ => throw new ArgumentOutOfRangeException(nameof(shape)),
    };

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
            if (h.Name.EndsWith(".x", StringComparison.Ordinal))
            {
                string key = h.Name[..^2];
                Vec2 center = P(key + ".c");
                float halfY = P(key + ".y").Distance(center);
                Vec2 axisX = (newPos - center).Normalized(Vec2.UnitX);
                h.Pos = newPos;
                _handles[key + ".y"].Pos = center + axisX.Perp * halfY;
                Invalidate();
                base.OnMouseMove(e);
                return;
            }
            if (h.Name.EndsWith(".y", StringComparison.Ordinal))
            {
                string key = h.Name[..^2];
                Vec2 center = P(key + ".c");
                Vec2 axisY = (P(key + ".x") - center).Normalized(Vec2.UnitX).Perp;
                float halfY = MathF.Max(6f, (newPos - center).Length);
                h.Pos = center + axisY * halfY;
                Invalidate();
                base.OnMouseMove(e);
                return;
            }
            var delta = newPos - h.Pos;
            h.Pos = newPos;
            foreach (var childName in h.Children)
                if (_handles.TryGetValue(childName, out var child))
                    child.Pos += delta;

            if (h.Name.EndsWith(".a", StringComparison.Ordinal)
                || h.Name.EndsWith(".b", StringComparison.Ordinal))
            {
                string key = h.Name[..^2];
                if (_handles.TryGetValue(key + ".c", out Handle? center)
                    && _handles.TryGetValue(key + ".a", out Handle? a)
                    && _handles.TryGetValue(key + ".b", out Handle? b))
                    center.Pos = (a.Pos + b.Pos) * .5f;
            }
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

        try
        {
            if (_genericSweep)
            {
                DrawGenericSweep(g);
            }
            else
            {
                switch (_scene)
                {
                    case SceneKind.CircleCircle: DrawDiscrete(g, GetCircle("A"), GetCircle("B")); break;
                    case SceneKind.CircleAabb: DrawDiscrete(g, GetCircle("A"), GetBox("B")); break;
                    case SceneKind.CircleCapsule: DrawDiscrete(g, GetCircle("A"), GetCapsule("B")); break;
                    case SceneKind.CircleObb: DrawDiscrete(g, GetCircle("A"), GetObb("B")); break;
                    case SceneKind.CirclePolygon: DrawDiscrete(g, GetCircle("A"), GetPolygon("B")); break;
                    case SceneKind.AabbAabb: DrawDiscrete(g, GetBox("A"), GetBox("B")); break;
                    case SceneKind.CapsuleAabb: DrawDiscrete(g, GetCapsule("A"), GetBox("B")); break;
                    case SceneKind.AabbObb: DrawDiscrete(g, GetBox("A"), GetObb("B")); break;
                    case SceneKind.AabbPolygon: DrawDiscrete(g, GetBox("A"), GetPolygon("B")); break;
                    case SceneKind.CapsuleCapsule: DrawDiscrete(g, GetCapsule("A"), GetCapsule("B")); break;
                    case SceneKind.CapsuleObb: DrawDiscrete(g, GetCapsule("A"), GetObb("B")); break;
                    case SceneKind.CapsulePolygon: DrawDiscrete(g, GetCapsule("A"), GetPolygon("B")); break;
                    case SceneKind.ObbObb: DrawDiscrete(g, GetObb("A"), GetObb("B")); break;
                    case SceneKind.ObbPolygon: DrawDiscrete(g, GetObb("A"), GetPolygon("B")); break;
                    case SceneKind.PolygonPolygon: DrawDiscrete(g, GetPolygon("A"), GetPolygon("B")); break;
                    case SceneKind.SweptCircleCircle: DrawSwept(g, isBox: false); break;
                    case SceneKind.SweptCircleAabb: DrawSwept(g, isBox: true); break;
                    case SceneKind.RayCircle: DrawRaySweep(g, isBox: false); break;
                    case SceneKind.RayAabb: DrawRaySweep(g, isBox: true); break;
                }
            }
        }
        catch (ArgumentException error)
        {
            InfoChanged?.Invoke($"Scene: {_scene}\n\ninvalid shape\n\n{error.Message}");
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
        Manifold m = Collide.ShapeVsShape(ToShape(a), ToShape(b));
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

    private static Shape ToShape(object value) => value switch
    {
        Circle shape => shape,
        Aabb shape => shape,
        Capsule shape => shape,
        Obb shape => shape,
        Polygon shape => shape,
        _ => throw new ArgumentOutOfRangeException(nameof(value)),
    };

    private void DrawSwept(Graphics g, bool isBox)
    {
        Circle mover = GetCircle("A");
        Vec2 motion = P("A.m") - mover.Center;

        SweepHit hit = isBox
            ? Sweep.MovingCircleVsAabb(mover, motion, GetBox("B"))
            : Sweep.MovingCircleVsCircle(mover, motion, GetCircle("B"));

        object target = isBox ? GetBox("B") : GetCircle("B");
        DrawShape(g, target, ColB);

        // motion path
        DrawArrow(g, mover.Center, P("A.m"), ColMotion, 2f);

        // start (solid) + end (faint) mover
        DrawCircleOutline(g, mover.Center, mover.Radius, ColA, 2f);
        DrawCircleOutline(g, mover.Center + motion, mover.Radius, Color.FromArgb(70, ColA), 1.5f);
        DrawLabel(g, mover.Center, "A start", ColA);
        DrawLabel(g, mover.Center + motion, "A end (ghost)", Color.FromArgb(180, ColA));
        DrawLabel(g, VisualCenter(target), "B target", ColB);

        if (hit.Hit)
        {
            Vec2 stop = mover.Center + motion * hit.Time;
            DrawCircleOutline(g, stop, mover.Radius, ColHit, 2.4f);
            DrawLabel(g, stop, "A at TOI", ColHit);
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

    private void DrawRaySweep(Graphics g, bool isBox)
    {
        Vec2 origin = P("A.o");
        Vec2 end = P("A.m");
        Vec2 displacement = end - origin;
        SweepHit hit = isBox
            ? Sweep.RayVsAabb(origin, displacement, GetBox("B"))
            : Sweep.RayVsCircle(origin, displacement, GetCircle("B"));

        object target = isBox ? GetBox("B") : GetCircle("B");
        DrawShape(g, target, ColB);
        DrawArrow(g, origin, end, ColMotion, 2f);
        DrawLabel(g, origin, "ray start", ColMotion);
        DrawLabel(g, end, "ray end", Color.FromArgb(180, ColMotion));
        DrawLabel(g, VisualCenter(target), "B target", ColB);

        if (hit.Hit)
        {
            FillDot(g, hit.Point, 5f, ColHit);
            using var impactPen = new Pen(Color.FromArgb(150, ColHit), 2f)
            {
                DashStyle = DashStyle.Dash,
            };
            g.DrawLine(impactPen, origin.X, origin.Y, hit.Point.X, hit.Point.Y);
            if (ShowContacts)
                DrawArrow(g, hit.Point, hit.Point + hit.Normal * 50f, ColNormal, 2.2f);
        }

        InfoChanged?.Invoke(
            $"Scene: {_scene}\n\n" +
            (hit.Hit
                ? $"IMPACT\n\ntime   : {hit.Time,7:0.000}\nnormal : ({hit.Normal.X,5:0.00}, {hit.Normal.Y,5:0.00})\npoint  : ({hit.Point.X,5:0.0}, {hit.Point.Y,5:0.0})"
                : "clear ray (no impact)"));
    }

    private void DrawGenericSweep(Graphics g)
    {
        object moverVisual = GetVisualShape("A", _sweepMoverKind);
        object targetVisual = GetVisualShape("B", _sweepTargetKind);
        Vec2 start = VisualCenter(moverVisual);
        Vec2 motion = P("A.m") - start;
        Shape moverShape = ToShape(moverVisual);
        Shape targetShape = ToShape(targetVisual);
        SweepAlgorithm algorithm = Sweep.GetAlgorithm(moverShape, targetShape);
        SweepHit hit = Sweep.MovingShapeVsShape(moverShape, motion, targetShape);

        object endVisual = MoveVisualShape(moverVisual, motion);
        DrawShape(g, targetVisual, ColB);
        DrawShape(g, moverVisual, ColA);
        DrawShape(g, endVisual, Color.FromArgb(70, ColA));
        DrawArrow(g, start, P("A.m"), ColMotion, 2f);
        DrawLabel(g, start, "A start", ColA);
        DrawLabel(g, VisualCenter(endVisual), "A end (ghost)", Color.FromArgb(180, ColA));
        DrawLabel(g, VisualCenter(targetVisual), "B target", ColB);

        if (hit.Hit)
        {
            object impactVisual = MoveVisualShape(moverVisual, motion * hit.Time);
            DrawShape(g, impactVisual, ColHit);
            DrawLabel(g, VisualCenter(impactVisual), "A at TOI", ColHit);
            FillDot(g, hit.Point, 5f, Color.White);
            if (ShowContacts)
                DrawArrow(g, hit.Point, hit.Point + hit.Normal * 50f, ColNormal, 2.2f);
        }

        InfoChanged?.Invoke(
            $"Swept: {_sweepMoverKind} vs {_sweepTargetKind}\n\n" +
            $"algorithm: {algorithm}\n\n" +
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
            case Obb box: DrawObb(g, box, color); break;
            case Polygon polygon: DrawPolygon(g, polygon, color); break;
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

    private static void DrawObb(Graphics g, Obb box, Color color)
    {
        float c = MathF.Cos(box.Rotation);
        float s = MathF.Sin(box.Rotation);
        Vec2 x = new(c * box.HalfExtents.X, s * box.HalfExtents.X);
        Vec2 y = new(-s * box.HalfExtents.Y, c * box.HalfExtents.Y);
        DrawPolygonPoints(g, new[]
        {
            box.Center - x - y,
            box.Center + x - y,
            box.Center + x + y,
            box.Center - x + y,
        }, color);
    }

    private static void DrawPolygon(Graphics g, Polygon polygon, Color color)
    {
        var points = new Vec2[polygon.Count];
        for (int i = 0; i < points.Length; i++) points[i] = polygon[i];
        DrawPolygonPoints(g, points, color);
    }

    private static void DrawPolygonPoints(Graphics g, Vec2[] vertices, Color color)
    {
        var points = new PointF[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            points[i] = new PointF(vertices[i].X, vertices[i].Y);
        using var fill = new SolidBrush(Color.FromArgb(60, color));
        using var pen = new Pen(color, 2f);
        g.FillPolygon(fill, points);
        g.DrawPolygon(pen, points);
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
        AddOuterSemicircle(path, cap.B, cap.Radius, Angle(n));
        path.AddLine(b2.X, b2.Y, a2.X, a2.Y);
        AddOuterSemicircle(path, cap.A, cap.Radius, Angle(-n));
        path.CloseFigure();

        g.FillPath(fill, path);
        g.DrawPath(pen, path);
        path.Dispose();
    }

    private static float Angle(Vec2 v) => MathF.Atan2(v.Y, v.X) * 180f / MathF.PI;

    private static void AddOuterSemicircle(
        GraphicsPath path, Vec2 center, float radius, float startDegrees)
    {
        // The path reaches each endpoint along one side of the spine. GDI+'s
        // positive sweep turns inward here; the exterior cap is -180 degrees.
        path.AddArc(center.X - radius, center.Y - radius,
            radius * 2, radius * 2, startDegrees, -180f);
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

    private static void DrawLabel(Graphics g, Vec2 anchor, string text, Color color)
    {
        var position = new PointF(anchor.X + 9f, anchor.Y + 9f);
        using var shadow = new SolidBrush(Color.FromArgb(190, 0, 0, 0));
        using var brush = new SolidBrush(Color.FromArgb(235, color));
        Font font = SystemFonts.MessageBoxFont!;
        g.DrawString(text, font, shadow,
            position.X + 1f, position.Y + 1f);
        g.DrawString(text, font, brush, position);
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
