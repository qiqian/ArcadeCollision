using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;

namespace ArcCollision.Battlefield;

/// <summary>
/// Renders Stage 01 from the source scene's Sprite2D and QuiverSpriteRepeater
/// values. Coordinates remain in the original 1920x1080 canvas and are mapped
/// once through <see cref="Game.WorldScale"/>.
/// </summary>
internal sealed class StageRenderer : IDisposable
{
    private readonly Dictionary<string, Image> _images = new(StringComparer.OrdinalIgnoreCase);
    private readonly Dictionary<int, Bitmap> _staticChunks = new();
    private float _cameraSourceX;
    private bool _renderingStaticChunk;
    private string Root => Path.Combine(AppContext.BaseDirectory, "Assets");
    private const int ChunkWidth = 1280;
    private const int CacheTop = -800;
    private const int CacheHeight = 1800;

    private static float X(float source) => source * Game.WorldScale;
    private static float Y(float source) => source * Game.WorldScale;

    public void Preload()
    {
        string stageRoot = Path.Combine(Root, "stages", "stage_01");
        if (Directory.Exists(stageRoot))
        {
            foreach (string path in Directory.GetFiles(
                stageRoot, "*.png", SearchOption.AllDirectories))
            {
                string relative = Path.GetRelativePath(Root, path)
                    .Replace(Path.DirectorySeparatorChar, '/');
                Load(relative);
            }
        }

        int chunkCount = (int)MathF.Ceiling(Game.WorldW / ChunkWidth);
        for (int index = 0; index < chunkCount; index++)
            GetStaticChunk(index);
    }

    public void DrawBackground(Graphics g, float cameraX, float zoom)
    {
        float cameraSourceX = cameraX / Game.WorldScale;
        _cameraSourceX = cameraSourceX;
        if (_renderingStaticChunk) goto StaticWorld;

        using (var sky = new LinearGradientBrush(new RectangleF(0, 0, Game.WorldW, 720),
                   Color.FromArgb(7, 12, 28), Color.FromArgb(225, 145, 132), LinearGradientMode.Vertical))
            g.FillRectangle(sky, 0, 0, Game.WorldW, 720);

        // stage_01/Skybox: a 1920x540 region at (-8,-444), scale 1.86482,
        // motion_scale.x=0. Draw it at the camera source x so it remains screen fixed.
        foreach (string layer in new[] {
            "area_1_skybox_1_0.png", "area_1_skybox_1_1.png", "area_1_skybox_1_2.png",
            "area_1_skybox_1_3.png", "area_1_skybox_1_5.png" })
            DrawTopLeft(g, Load("stages/stage_01/stage_elements/skyboxes/" + layer),
                cameraSourceX - 8f, -444f, 1.86482f);

        // stage_01.tscn: SkyboxLayer/ParallaxLayer2/Sprite2d
        Sprite(g, "stages/stage_01/stage_elements/buildings/cityscape_00.png",
            3730 + cameraSourceX * .35f, 35.25f, .678503f);

        DrawStaticChunks(g, cameraX, zoom);
        return;

StaticWorld:
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_back_brick_wall_dirty.png", 9937, -37);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_tree_leftside.png", 11770, 321, 1.0194f);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_tree_rightside.png", 16077, 366, 1.0194f);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_windows.png", 13754.8f, 259.75f, 1.0194f);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_black_marble.png", 13251, 820);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_floor_chess.png", 14211, 839);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_pilar_leftside.png", 10882, 74);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_pilar_leftside.png", 11834, 74);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_pilar_leftside.png", 15946, 74);
        foreach (var p in new[] {
            new PointF(11075,282), new PointF(12022,282), new PointF(11695,282),
            new PointF(15784,282), new PointF(16030,363) })
            Sprite(g, "stages/stage_01/stage_elements/decorations/area_2_bamboo1_2.png", p.X, p.Y);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_tree_rightside.png", 6443, 99);

        Sprite(g, "stages/stage_01/stage_elements/buildings/area_2_fire_tiger_steakhouse_all.png", 9104, 110);
        Sprite(g, "stages/stage_01/stage_elements/buildings/area_2_java_bean_dream_all.png", 8470, 110);
        Sprite(g, "stages/stage_01/stage_elements/buildings/foster_mart/area_2_foster_mart_store_all.png", 7467, 110);
        Sprite(g, "stages/stage_01/stage_elements/buildings/foster_mart/area_2_rocks_revised.png", 6583, 364);
        Sprite(g, "stages/stage_01/stage_elements/buildings/foster_mart/area_2_foster_mart_sign.png", 6583, 206);

        DrawChickenKing(g, 2523, 506);
        DrawBrickDistrict(g);
        DrawFence(g);
        DrawDirtyGateBack(g);
        DrawToriBack(g);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_throne_revised.png",
            15599.001f, 535.75f, .75f);

        SpriteTopLeft(g, "stages/stage_01/stage_elements/pavement/street_revised_curved.png", -15, 650);
        Repeat(g, "stages/stage_01/stage_elements/pavement/street_revised_straight.png",
            568, 650, 6, -480, false);
        SpriteTopLeft(g, "stages/stage_01/stage_elements/pavement/area_2_crosswalk.png", 2993, 650);
        SpriteTopLeft(g, "stages/stage_01/stage_elements/pavement/area_2_crosswalk.png", 6836, 650);
        SpriteTopLeft(g, "stages/stage_01/stage_elements/pavement/tax_man_streetlines.png", 9784, 638);

        RepeatSidewalk(g, 439.363f, 541.217f, 49,
            new[] { 0,0,0,0,0,1,1,1,1,0,1,1,1,0,0,0,0,0,1,1,1,1,0,0,2,0,1,1,0,0,0,0,2,2,0,0,1,0,2,2,1,0,0,0,1,1,0,1,0 });
        RepeatSidewalk(g, 1842.36f, 482.217f, 42,
            new[] { 1,1,0,2,2,1,1,0,0,1,2,2,1,1,1,1,1,1,2,0,1,1,2,1,1,1,1,1,2,1,1,1,0,0,1,1,1,0,0,1,0,2 });
        RepeatSidewalk(g, 9308.36f, 424.217f, 6, new[] { 1,2,2,0,2,1 });

        Sprite(g, "stages/stage_01/stage_elements/vending_machines/vending_machines_clean_separated_yellow_50.png", 3880.25f, 375.856f, 1.21205f);
        Sprite(g, "stages/stage_01/stage_elements/vending_machines/vending_machine_tags_separated_3_50.png", 3737.25f, 374.856f, 1.21205f);
        Sprite(g, "stages/stage_01/stage_elements/vending_machines/vending_machines_clean_separated_red_50.png", 3553.25f, 374.856f, 1.21205f);

        Sprite(g, "stages/stage_01/stage_elements/street_poles/telephone_pole_revised_1_50.png", 1351, 19);
        Sprite(g, "stages/stage_01/stage_elements/street_poles/telephone_pole_revised_1_50.png", 5531, 19);
        foreach (float x in new[] { 1280f, 3373.363f, 5465.003f, 7557.003f, 9641.003f })
            Sprite(g, "stages/stage_01/stage_elements/street_poles/lamp_poles/streetlamp_revised_2_01_50.png", x, 83);
        Sprite(g, "stages/stage_01/stage_elements/street_poles/post_box/postbox_tagged.png", 1401, 484);
        Sprite(g, "stages/stage_01/stage_elements/street_poles/post_box/postbox_clean.png", 4418, 484);
        foreach (float x in new[] { 6998f, 7163f, 7328f, 7703f, 7868f, 8033f, 8198f, 8363f, 8528f, 8693f, 8858f, 9023f })
            Sprite(g, "stages/stage_01/stage_elements/street_poles/metal_post.png", x, 404, 1f);
    }

    private void DrawStaticChunks(Graphics g, float cameraX, float zoom)
    {
        float visibleWidth = Game.ArenaW / MathF.Max(.1f, zoom);
        int first = Math.Max(0, (int)MathF.Floor(cameraX / ChunkWidth));
        int last = Math.Min((int)MathF.Ceiling(Game.WorldW / ChunkWidth) - 1,
            (int)MathF.Floor((cameraX + visibleWidth) / ChunkWidth));
        for (int index = first; index <= last; index++)
        {
            Bitmap chunk = GetStaticChunk(index);
            g.DrawImage(chunk, index * ChunkWidth, CacheTop, ChunkWidth, CacheHeight);
        }
    }

    private Bitmap GetStaticChunk(int index)
    {
        if (_staticChunks.TryGetValue(index, out Bitmap? cached)) return cached;

        var chunk = new Bitmap(ChunkWidth, CacheHeight,
            System.Drawing.Imaging.PixelFormat.Format32bppPArgb);
        using (Graphics graphics = Graphics.FromImage(chunk))
        {
            graphics.Clear(Color.Transparent);
            graphics.CompositingQuality = CompositingQuality.HighSpeed;
            graphics.InterpolationMode = InterpolationMode.NearestNeighbor;
            graphics.PixelOffsetMode = PixelOffsetMode.Half;
            graphics.TranslateTransform(-index * ChunkWidth, -CacheTop);
            _renderingStaticChunk = true;
            try
            {
                DrawBackground(graphics, index * ChunkWidth, 1f);
            }
            finally
            {
                _renderingStaticChunk = false;
            }
        }
        _staticChunks.Add(index, chunk);
        return chunk;
    }

    public void DrawForeground(Graphics g)
    {
        DrawDirtyGateFront(g);
        DrawToriFront(g);
        Sprite(g, "stages/stage_01/stage_elements/street_poles/telephone_pole_revised_1_50.png", 611, 281, 2f);
        Sprite(g, "stages/stage_01/stage_elements/street_poles/telephone_pole_revised_1_50.png", 6633, 281, 2f);
    }

    private void DrawChickenKing(Graphics g, float x, float y)
    {
        Sprite(g, "stages/stage_01/stage_elements/buildings/chicken_king/pngs/chicken_king_base.png", x, y - 233.5f);
        Sprite(g, "stages/stage_01/stage_elements/buildings/chicken_king/pngs/chicken_king_window_covers.png", x + 9, y - 194);
        Sprite(g, "stages/stage_01/stage_elements/buildings/chicken_king/pngs/chicken_king_front_banner.png", x, y - 398);
        Sprite(g, "stages/stage_01/stage_elements/buildings/chicken_king/pngs/chicken_king_top.png", x, y - 730.5f);
        Sprite(g, "stages/stage_01/stage_elements/buildings/chicken_king/pngs/chicken_king_billboard.png", x, y - 1144);
        Sprite(g, "stages/stage_01/stage_elements/buildings/chicken_king/pngs/spot_lights.png", x - 2, y - 885);
        DrawTopLeft(g, Load("stages/stage_01/stage_elements/decorations/christmas_lights/christmas_lights_1_50.png"),
            x - 543, y - 623, 1.49074f);
        DrawTopLeft(g, Load("stages/stage_01/stage_elements/decorations/lantern/lanterns_1_50.png"),
            x - 574, y - 862, 1.40444f);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/tax_man_tree_leftside.png", x + 635, y - 509);
    }

    private void DrawBrickDistrict(Graphics g)
    {
        // The source BrickWall scene is a region-filled texture. Repeat its exact source tile.
        TileRegion(g, "stages/stage_01/stage_elements/buildings/walls/brick_wall/brick_pattern_grayscale.png",
            509.5f, 15, 1073, 1100, .5f);
        TileRegion(g, "stages/stage_01/stage_elements/buildings/walls/brick_wall/brick_pattern_grayscale.png",
            1550.2f, 13.5f, 921.2f, 1103, .5f);
        Sprite(g, "stages/stage_01/stage_elements/buildings/doors/door_2_revised_50.png", 217, 358);
        Sprite(g, "stages/stage_01/stage_elements/buildings/doors/door_3_50.png", 680, 358);
        Sprite(g, "stages/stage_01/stage_elements/buildings/signs/anime_girl/anime_girl_poster_red.png", 468.5f, 328.575f, .81875f);
        Sprite(g, "stages/stage_01/stage_elements/buildings/signs/anime_girl/anime_girl_poster_purlple.png", 891.5f, 328.575f, .81875f);
        Sprite(g, "stages/stage_01/stage_elements/buildings/signs/love_yourself/pngs/love_yourself_sign_base.png", 671, -27);
        Sprite(g, "stages/stage_01/stage_elements/buildings/signs/love_yourself/pngs/love_yourself_sign_lights_base.png", 671, -27);

        Repeat(g, "stages/stage_01/stage_elements/buildings/pipes/drainpipe_body.png",
            1003, -557, 3, -38, true, .5f);
        Sprite(g, "stages/stage_01/stage_elements/buildings/pipes/drainpipe_exit.png", 1006.5f, -376.5f, .5f);
        Repeat(g, "stages/stage_01/stage_elements/buildings/windows/standard/windows_1_1_50.png",
            -284, -467, 2, 180, false);

        Sprite(g, "stages/stage_01/stage_elements/buildings/doors/door_1_revised_new_50.png", 1842, 335);
        Sprite(g, "stages/stage_01/stage_elements/buildings/windows/narrow/window_2.png", 1842, -213);
        Repeat(g, "stages/stage_01/stage_elements/buildings/pipes/drainpipe_body.png",
            1960, -564.905f, 3, -38, true, .5f);
        Sprite(g, "stages/stage_01/stage_elements/buildings/pipes/drainpipe_exit.png", 1963.5f, -384.405f, .5f);
        Repeat(g, "stages/stage_01/stage_elements/buildings/windows/standard/windows_1_3_50.png",
            1139, -360, 2, 100, true);
    }

    private void DrawFence(Graphics g)
    {
        const float x = 3222, y = 22.383f, scale = .5f;
        Sprite(g, "stages/stage_01/stage_elements/fence/chainlink_fence_revised__01.png", x - 62, y, scale);
        Image middle = Load("stages/stage_01/stage_elements/fence/chainlink_fence_revised__00.png");
        float step = Math.Max(1, middle.Width * scale);
        for (int i = 0; i < 75; i++)
            DrawTopLeft(g, middle, x + i * step, y, scale);
        Sprite(g, "stages/stage_01/stage_elements/fence/chainlink_fence_revised__02.png", x + 75 * step, y, scale);
    }

    private void DrawDirtyGateBack(Graphics g)
    {
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/brick_gate/brick_gate_back.png", 10730.5f, 267.2f);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/brick_gate/tax_man_doors_1.png", 10749, 342);
    }

    private void DrawDirtyGateFront(Graphics g)
    {
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/brick_gate/tax_man_doors_2.png", 10994, 585);
        Sprite(g, "stages/stage_01/stage_elements/tax_man_area/brick_gate/brick_gate_front.png", 11546.1f, 404.51f);
    }

    private void DrawToriBack(Graphics g) =>
        Sprite(g, "stages/stage_01/stage_elements/gates/tori_gate/tori_gate_back.png",
            10005.75f, 3f, .75f);

    private void DrawToriFront(Graphics g) =>
        Sprite(g, "stages/stage_01/stage_elements/gates/tori_gate/tori_gate_front.png",
            10355.75f, 288.75f, .75f);

    private void RepeatSidewalk(Graphics g, float x, float y, int length, int[] sequence)
    {
        string[] textures = {
            "stages/stage_01/stage_elements/sidewalk/sidewalk_revised_0.png",
            "stages/stage_01/stage_elements/sidewalk/sidewalk_revised_1.png",
            "stages/stage_01/stage_elements/sidewalk/sidewalk_revised_2.png"
        };
        for (int i = 0; i < length; i++)
        {
            Image image = Load(textures[sequence[Math.Min(i, sequence.Length - 1)]]);
            DrawTopLeft(g, image, x + (image.Width - 99) * i, y, 1f);
        }
    }

    private void Repeat(Graphics g, string path, float x, float y, int length, float separation,
        bool vertical, float scale = 1f)
    {
        Image image = Load(path);
        for (int i = 0; i < length; i++)
            DrawTopLeft(g, image, x + (vertical ? 0 : (image.Width * scale + separation) * i),
                y + (vertical ? (image.Height * scale + separation) * i : 0), scale);
    }

    private void TileRegion(Graphics g, string path, float x, float y, float width, float height, float nodeScale)
    {
        Image image = Load(path);
        float stepX = image.Width * nodeScale, stepY = image.Height * nodeScale;
        for (float yy = y; yy < y + height; yy += stepY)
            for (float xx = x; xx < x + width; xx += stepX)
                DrawTopLeft(g, image, xx, yy, nodeScale);
    }

    private void Sprite(Graphics g, string path, float x, float y, float scale = 1f)
    {
        if (!_renderingStaticChunk
            && (x < _cameraSourceX - 4000f || x > _cameraSourceX + 6000f)) return;
        Image image = Load(path);
        float w = image.Width * scale * Game.WorldScale;
        float h = image.Height * scale * Game.WorldScale;
        g.DrawImage(image, X(x) - w / 2, Y(y) - h / 2, w, h);
    }

    private void SpriteTopLeft(Graphics g, string path, float x, float y) => DrawTopLeft(g, Load(path), x, y, 1f);

    private void DrawTopLeft(Graphics g, Image image, float x, float y, float scale)
    {
        float sourceWidth = image.Width * scale;
        if (!_renderingStaticChunk
            && (x + sourceWidth < _cameraSourceX - 300f || x > _cameraSourceX + 2220f)) return;
        float w = image.Width * scale * Game.WorldScale;
        float h = image.Height * scale * Game.WorldScale;
        g.DrawImage(image, X(x), Y(y), w, h);
    }

    private Image Load(string relative)
    {
        if (_images.TryGetValue(relative, out Image? image)) return image;
        string path = Path.Combine(Root, relative.Replace('/', Path.DirectorySeparatorChar));
        image = File.Exists(path) ? Image.FromFile(path) : new Bitmap(1, 1);
        _images.Add(relative, image);
        return image;
    }

    public void Dispose()
    {
        foreach (Bitmap chunk in _staticChunks.Values) chunk.Dispose();
        _staticChunks.Clear();
        foreach (Image image in _images.Values) image.Dispose();
        _images.Clear();
    }
}
