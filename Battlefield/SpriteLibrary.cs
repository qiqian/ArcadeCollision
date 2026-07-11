using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;

namespace ArcCollision.Battlefield;

/// <summary>One animation clip: ordered frames + playback data + the sprite-space
/// centre offset (from the Quiver animation .tres files) used to keep frames of
/// different canvas sizes aligned to the character's feet.</summary>
internal sealed class Clip
{
    public required Bitmap[] Frames;
    public float Fps;
    public bool Loop;
    public float OffX;   // sprite-px offset of sprite centre from feet (x right)
    public float OffY;   // sprite-px offset (y up is negative, Godot convention)
    public bool BottomAnchor;  // when true, ignore OffX/OffY and anchor bottom-centre at the feet
    public bool FacesLeft;     // sprite art faces left by default (mirror to face right)
    public (float Time, float X, float Y)[] PositionKeys = Array.Empty<(float, float, float)>();

    public int FrameAt(float time)
    {
        if (Frames.Length <= 1) return 0;
        int i = (int)(time * Fps);
        return Loop ? i % Frames.Length : Math.Min(i, Frames.Length - 1);
    }

    public (float X, float Y) OffsetAt(float time)
    {
        float x = OffX, y = OffY;
        foreach (var key in PositionKeys)
        {
            if (time + 0.0001f < key.Time) break;
            x = key.X;
            y = key.Y;
        }
        return (x, y);
    }
}

/// <summary>
/// Loads and caches the chad / sarge sprite sets copied from the Quiver
/// beat-'em-up template. Frame ordering, fps and centre offsets are taken from
/// the original SpriteFrames and animation resources.
/// </summary>
internal static class SpriteLibrary
{
    private static readonly Dictionary<string, Dictionary<string, Clip>> _sets = new();
    private static string TemplateRoot => Path.Combine(AppContext.BaseDirectory, "Template");
    private static string FallbackRoot => Path.Combine(AppContext.BaseDirectory, "Assets");

    private static string SetRoot(string set) => set switch
    {
        "chad" => Path.Combine(TemplateRoot, "characters", "playable", "chad", "resources", "sprites"),
        "taxman" => Path.Combine(TemplateRoot, "characters", "enemies", "tax_man", "resources", "sprites"),
        _ => Path.Combine(TemplateRoot, "characters", "enemies", "sargent", "resources", "sprites"),
    };

    public static Dictionary<string, Clip> Get(string set)
    {
        if (_sets.TryGetValue(set, out var cached))
            return cached;
        var clips = set switch
        {
            "chad" => BuildChad(),
            "taxman" => BuildTaxman(),
            _ => BuildSarge(),
        };
        _sets[set] = clips;
        return clips;
    }

    private static Clip CB(Bitmap[] frames, float fps, bool loop, bool facesLeft = false) =>
        new() { Frames = frames, Fps = fps, Loop = loop, BottomAnchor = true, FacesLeft = facesLeft };

    private static Bitmap[] Frames(string set, params string[] rel)
    {
        var list = new List<Bitmap>(rel.Length);
        foreach (var r in rel)
        {
            string relative = r.Replace('/', Path.DirectorySeparatorChar) + ".png";
            string path = Path.Combine(SetRoot(set), relative);
            if (!File.Exists(path))
                path = Path.Combine(FallbackRoot, set, relative);
            if (File.Exists(path))
                list.Add(new Bitmap(path));
        }
        if (list.Count == 0)
            list.Add(new Bitmap(1, 1)); // guard against a totally missing clip
        return list.ToArray();
    }

    private static Bitmap[] FolderFrames(string set, string relativeFolder)
    {
        string folder = Path.Combine(SetRoot(set), relativeFolder.Replace('/', Path.DirectorySeparatorChar));
        if (!Directory.Exists(folder))
            folder = Path.Combine(FallbackRoot, set, relativeFolder.Replace('/', Path.DirectorySeparatorChar));
        if (!Directory.Exists(folder)) return new[] { new Bitmap(1, 1) };
        string[] files = Directory.GetFiles(folder, "*.png");
        Array.Sort(files, StringComparer.OrdinalIgnoreCase);
        var frames = new List<Bitmap>(files.Length);
        foreach (string file in files)
            try { frames.Add(new Bitmap(file)); } catch { }
        return frames.Count == 0 ? new[] { new Bitmap(1, 1) } : frames.ToArray();
    }

    private static Clip C(Bitmap[] frames, float fps, bool loop, float ox, float oy,
        bool facesLeft = false, params (float Time, float X, float Y)[] positionKeys) =>
        new()
        {
            Frames = frames, Fps = fps, Loop = loop, OffX = ox, OffY = oy,
            FacesLeft = facesLeft, PositionKeys = positionKeys
        };

    private static Clip CS(string set, float fps, bool loop, float ox, float oy,
        params (string Path, int Repeat)[] source)
    {
        var frames = new List<Bitmap>();
        foreach (var item in source)
        {
            Bitmap frame = Frames(set, item.Path)[0];
            for (int i = 0; i < item.Repeat; i++) frames.Add(frame);
        }
        return C(frames.ToArray(), fps, loop, ox, oy);
    }

    private static Clip CSS(string set, float fps, bool loop, float ox, float oy, bool facesLeft,
        (float Time, float X, float Y)[] keys, params (string Path, int Repeat)[] source)
    {
        Clip clip = CS(set, fps, loop, ox, oy, source);
        clip.FacesLeft = facesLeft;
        clip.PositionKeys = keys;
        return clip;
    }

    private static Dictionary<string, Clip> BuildChad()
    {
        const string s = "chad";
        return new Dictionary<string, Clip>
        {
            ["idle"] = C(Frames(s, "idle/idle_00", "idle/idle_01", "idle/idle_02", "idle/idle_03"), 8, true, 0, -167),
            ["walk"] = CSS(s, 24, true, 0, -190, false, Array.Empty<(float,float,float)>(),
                ("walk/walk_00",2), ("walk/walk_01",3), ("walk/walk_02",2), ("walk/walk_03",3),
                ("walk/walk_04",2), ("walk/walk_05",2), ("walk/walk_06",3), ("walk/walk_07",2),
                ("walk/walk_08",3), ("walk/walk_09",2), ("walk/walk_10",2), ("walk/walk_11",3)),
            ["turn"] = C(Frames(s, "turn_around/turnaround_00", "turn_around/turnaround_01", "turn_around/turnaround_02"), 24, false, 0, -190),
            ["attack1"] = CS(s, 24, false, 53, -164, ("punches/punch1_00",4), ("punches/punch1_01",2)),
            ["attack2"] = CSS(s, 24, false, 80, -177, false,
                new[] { (0f,80f,-177f), (.125f,80f,-170f), (.333216f,78f,-190f), (.333334f,53f,-163f) },
                ("punches/punch2_00",3), ("punches/punch2_01",5), ("punches/punch1_01",3)),
            ["attack3"] = CSS(s, 24, false, 80, -277, false,
                new[] { (0f,80f,-277f), (.583334f,53f,-165f) },
                ("punches/punch3_00",5), ("punches/punch3_01",3), ("punches/punch3_02",2),
                ("punches/punch3_04",4), ("punches/punch1_01",3)),
            ["jump"] = CS(s, 24, false, 12, -230, ("jump/jump_01",2), ("jump/jump_02",5)),
            ["rising"] = C(Frames(s, "jump/jump_03"), 24, false, -16, -222),
            ["falling"] = C(Frames(s, "jump/jump_04"), 24, false, -19, -264),
            ["landing"] = CS(s, 24, false, 0, -219, ("jump/jump_02",3), ("jump/jump_01",2)),
            ["air_attack"] = CS(s, 24, false, 101, -212, ("air_attack/air_attack_00",3), ("air_attack/air_attack_01",1)),
            ["hurt_mid"] = C(Frames(s, "hurt/hurt_mid"), 24, false, -16, -166),
            ["hurt_high"] = C(Frames(s, "hurt/hurt_high"), 24, false, -36, -190),
            ["ko_launch"] = C(Frames(s, "knock_out/knockout_00"), 24, false, 20, -228),
            ["ko_rising"] = C(Frames(s, "knock_out/knockout_01"), 24, false, -51, -218),
            ["ko_falling"] = C(Frames(s, "knock_out/knockout_02"), 24, false, 47, -211),
            ["ko_bounce"] = C(Frames(s, "knock_out/knockout_03", "knock_out/knockout_04"), 24, false, 18, -157),
            ["ko_landed"] = C(Frames(s, "knock_out/knockout_05"), 24, true, -34, -38),
            ["getup"] = CS(s, 24, false, 0, -227, ("jump/jump_02",3), ("jump/jump_01",2)),
            ["death"] = C(Frames(s, "knock_out/knockout_05"), 24, false, -34, -66),
        };
    }

    private static Dictionary<string, Clip> BuildSarge()
    {
        const string s = "sarge";
        return new Dictionary<string, Clip>
        {
            ["idle"] = C(Frames(s, "idle/idle_00", "idle/idle_01", "idle/idle_02", "idle/idle_03"), 4, true, 0, -216, true),
            ["walk"] = CSS(s, 24, true, 0, -239, false, Array.Empty<(float,float,float)>(),
                ("walk/walk_00",3), ("walk/walk_01",3), ("walk/walk_02",4), ("walk/walk_03",3),
                ("walk/walk_04",2), ("walk/walk_05",3), ("walk/walk_06",4), ("walk/walk_07",3),
                ("walk/walk_08",3), ("walk/walk_09",3), ("walk/walk_10",3), ("walk/walk_11",4)),
            ["turn"] = CS(s, 24, false, 0, -239,
                ("turn_around/turnaround_00",2), ("turn_around/turnaround_01",2), ("turn_around/turnaround_02",2)),
            ["attack1"] = CS(s, 24, false, 32, -216, ("attacks/punch_1/punch_1_00",2), ("attacks/punch_1/punch_1_01",2), ("attacks/punch_1/punch_1_02",2), ("attacks/punch_1/punch_1_03",2)),
            ["attack2"] = CS(s, 24, false, 56, -216, ("attacks/punch_2/punch_2_01",1), ("attacks/punch_2/punch_2_02",2), ("attacks/punch_2/punch_2_03",3), ("attacks/punch_2/punch_2_04",1), ("attacks/punch_2/punch_2_05",3), ("attacks/punch_2/punch_2_06",1), ("attacks/punch_2/punch_2_07",1), ("attacks/punch_2/punch_2_08",2), ("attacks/punch_2/punch_2_09",1)),
            ["attack3"] = CSS(s, 24, false, -202, -216, false,
                new[] { (0f,-202f,-216f), (.291667f,-283f,-212f) },
                ("attacks/punch_3/punch_3_00",5), ("attacks/punch_3/punch_3_01",2),
                ("attacks/punch_3/punch_3_02",7), ("attacks/punch_3/punch_3_03",10),
                ("attacks/punch_3/punch_3_04",2), ("attacks/punch_3/punch_3_05",3),
                ("attacks/punch_3/punch_3_06",5)),
            ["jump"] = CS(s, 24, false, 33, -333, ("jump/jump_01",2), ("jump/jump_02",5)),
            ["rising"] = CS(s, 24, false, 0, -304, ("jump/jump_03",2), ("jump/jump_04",1)),
            ["falling"] = C(Frames(s, "jump/jump_05"), 24, false, 0, -285),
            ["landing"] = CSS(s, 24, false, 0, -328, false,
                new[] { (0f,0f,-328f), (.291667f,32f,-333f) },
                ("jump/jump_02",2), ("jump/jump_06",5), ("jump/jump_01",5),
                ("jump/jump_07",5), ("jump/jump_08",2)),
            ["hurt_mid"] = C(Frames(s, "hurt/injured_00"), 24, false, 0, -206),
            ["hurt_high"] = C(Frames(s, "hurt/injured_01"), 24, false, 0, -209),
            ["ko_launch"] = C(Frames(s, "knockout/ko_00"), 24, false, 0, -261),
            ["ko_rising"] = C(Frames(s, "knockout/ko_01"), 24, false, -83, -239),
            ["ko_falling"] = C(Frames(s, "knockout/ko_02"), 24, false, 0, -258),
            ["ko_bounce"] = CSS(s, 24, false, 0, -196, false, Array.Empty<(float,float,float)>(),
                ("knockout/ko_03",2), ("knockout/ko_04",2)),
            ["ko_landed"] = C(Frames(s, "knockout/ko_05"), 24, false, 0, -35),
            ["getup"] = C(Frames(s, "jump/jump_06", "jump/jump_01", "jump/jump_07", "jump/jump_08"), 6, false, 32, -333),
            ["death"] = C(Frames(s, "knockout/ko_05"), 24, false, 0, -50),
        };
    }

    private static Dictionary<string, Clip> BuildTaxman()
    {
        const string s = "taxman";
        return new Dictionary<string, Clip>
        {
            ["idle"] = C(Frames(s, "idle/idle_00", "idle/idle_01", "idle/idle_02", "idle/idle_03",
                "idle/idle_04", "idle/idle_05", "idle/idle_06", "idle/idle_07"), 4, true, 0, -249, true),
            ["walk"] = CSS(s, 24, true, 0, -250, true, Array.Empty<(float,float,float)>(),
                ("walk/walk_00",2), ("walk/walk_01",2), ("walk/walk_02",2), ("walk/walk_03",1),
                ("walk/walk_04",2), ("walk/walk_05",1), ("walk/walk_06",2), ("walk/walk_07",2),
                ("walk/walk_08",2), ("walk/walk_09",2), ("walk/walk_10",1), ("walk/walk_11",2),
                ("walk/walk_12",1), ("walk/walk_13",2), ("walk/walk_14",2), ("walk/walk_15",1)),
            ["turn"] = C(Frames(s, "turn/turnaround_02", "turn/turnaround_01", "turn/turnaround_00"), 24, false, 0, -248),
            ["retaliate"] = CSS(s, 24, false, 85, -262, true,
                new[] { (0f,85f,-262f), (.166667f,85f,-269f), (.25f,37f,-288f) },
                ("attacks/slap/slap_2_01",4), ("attacks/slap/slap_2_02",2),
                ("attacks/slap/slap_2_04",2), ("attacks/slap/slap_2_05",4)),
            ["attack1"] = CSS(s, 24, false, 85, -262, true,
                new[] { (0f,85f,-262f), (.166667f,85f,-269f), (.25f,37f,-288f) },
                ("attacks/slap/slap_2_01",4), ("attacks/slap/slap_2_02",2),
                ("attacks/slap/slap_2_04",2), ("attacks/slap/slap_2_05",4)),
            ["attack_combo"] = CSS(s, 24, false, 0, -267, true,
                new[] { (0f,0f,-267f), (.0833333f,-144f,-261f) },
                ("attacks/combo_sequence/attack_01",2), ("attacks/combo_sequence/attack_02_blur",2),
                ("attacks/combo_sequence/attack_03_blur",2), ("attacks/combo_sequence/attack_03",3),
                ("attacks/combo_sequence/attack_04",1), ("attacks/combo_sequence/attack_05_blur",2),
                ("attacks/combo_sequence/attack_05",4), ("attacks/combo_sequence/attack_06",1),
                ("attacks/combo_sequence/attack_07",2), ("attacks/combo_sequence/attack_08",1),
                ("attacks/combo_sequence/attack_09",3), ("attacks/combo_sequence/attack_10",3),
                ("attacks/combo_sequence/attack_11_blur",2), ("attacks/combo_sequence/attack_12_blur",2),
                ("attacks/combo_sequence/attack_12",3), ("attacks/combo_sequence/attack_13",2),
                ("attacks/combo_sequence/attack_14",5)),
            ["attack_area"] = CSS(s, 24, false, 160, -227, true,
                new[] {
                    (0f,160f,-227f), (.125f,104f,-227f), (1.79167f,16f,-227f),
                    (2.04167f,-16f,-247f), (2.125f,-48f,-231f), (2.5f,32f,-275f),
                    (3.33333f,0f,-249f)
                },
                ("attacks/area_attack/tax_man/area_attack0000",2),
                ("attacks/area_attack/tax_man/area_attack0002",2),
                ("attacks/area_attack/tax_man/area_attack0004",2),
                ("attacks/area_attack/tax_man/area_attack0006",3),
                ("attacks/area_attack/tax_man/area_attack0009",1),
                ("attacks/area_attack/tax_man/area_attack0010",1),
                ("attacks/area_attack/tax_man/area_attack0011",2),
                ("attacks/area_attack/tax_man/area_attack0013",1),
                ("attacks/area_attack/tax_man/area_attack0014",1),
                ("attacks/area_attack/tax_man/area_attack0015",3),
                ("attacks/area_attack/tax_man/area_attack0018",2),
                ("attacks/area_attack/tax_man/area_attack0020",2),
                ("attacks/area_attack/tax_man/area_attack0022",2),
                ("attacks/area_attack/tax_man/area_attack0024",1),
                ("attacks/area_attack/tax_man/area_attack0025",1),
                ("attacks/area_attack/tax_man/area_attack0026",3),
                ("attacks/area_attack/tax_man/area_attack0029",1),
                ("attacks/area_attack/tax_man/area_attack0030",2),
                ("attacks/area_attack/tax_man/area_attack0032",2),
                ("attacks/area_attack/tax_man/area_attack0034",2),
                ("attacks/area_attack/tax_man/area_attack0036",2),
                ("attacks/area_attack/tax_man/area_attack0038",2),
                ("attacks/area_attack/tax_man/area_attack0040",2),
                ("attacks/area_attack/tax_man/area_attack0042",2),
                ("attacks/area_attack/tax_man/area_attack0044",2),
                ("attacks/area_attack/tax_man/area_attack0046",2),
                ("attacks/area_attack/tax_man/area_attack0048",2),
                ("attacks/area_attack/tax_man/area_attack0050",3),
                ("attacks/area_attack/tax_man/area_attack0053",4),
                ("attacks/area_attack/tax_man/area_attack0057",1)),
            ["attack_dash"] = CSS(s, 12, false, -816, -431, true,
                new[] { (0f,-816f,-431f), (.833333f,-824f,-433f) },
                ("attacks/dash_attack/dash_00",1), ("attacks/dash_attack/dash_01",1),
                ("attacks/dash_attack/dash_02",2), ("attacks/dash_attack/dash_04",2),
                ("attacks/dash_attack/dash_06",2), ("attacks/dash_attack/dash_08",1),
                ("attacks/dash_attack/dash_09",1), ("attacks/dash_attack/dash_10",1),
                ("attacks/dash_attack/dash_11",1), ("attacks/dash_attack/dash_13",1),
                ("attacks/dash_attack/dash_14",1), ("attacks/dash_attack/dash_15",1),
                ("attacks/dash_attack/dash_16",1), ("attacks/dash_attack/dash_17",1),
                ("attacks/dash_attack/dash_18",1)),
            ["hurt_mid"] = C(Frames(s, "hurt/injury_small"), 24, false, -55, -240, true),
            ["hurt_high"] = C(Frames(s, "hurt/injury_medium"), 24, false, 0, -247, true),
            ["hurt_knockout"] = C(Frames(s, "knockout/injury_knockout_impact"), 24, false, 0, -255, true),
            ["ko_launch"] = C(Frames(s, "knockout/injury_knockout_impact"), 24, false, 0, -255, true),
            ["ko_rising"] = C(Frames(s, "knockout/injury_knockout_impact"), 24, false, 0, -255, true),
            ["ko_falling"] = C(Frames(s, "knockout/injury_knockout_impact"), 24, false, 0, -255, true),
            ["ko_bounce"] = C(Frames(s, "knockout/knock_out"), 5, true, -72, -258, true),
            ["ko_landed"] = C(Frames(s, "knockout/knock_out"), 5, true, -72, -258, true),
            ["getup"] = C(Frames(s, "knockout/knock_out"), 5, true, -72, -258, true),
            ["death"] = CSS(s, 24, false, 176, -277, true,
                new[] { (0f,176f,-277f), (.958334f,4f,-277f), (1.83333f,-108f,-277f), (2.5f,-12f,-277f) },
                ("attacks/grenade_death/tax_man/grenade_finisher0000",5),
                ("attacks/grenade_death/tax_man/grenade_finisher0005",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0007",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0005",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0007",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0005",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0007",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0005",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0007",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0005",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0023",5),
                ("attacks/grenade_death/tax_man/grenade_finisher0028",7),
                ("attacks/grenade_death/tax_man/grenade_finisher0035",3),
                ("attacks/grenade_death/tax_man/grenade_finisher0038",3),
                ("attacks/grenade_death/tax_man/grenade_finisher0041",3),
                ("attacks/grenade_death/tax_man/grenade_finisher0044",5),
                ("attacks/grenade_death/tax_man/grenade_finisher0049",7),
                ("attacks/grenade_death/tax_man/grenade_finisher0056",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0058",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0060",1),
                ("attacks/grenade_death/tax_man/grenade_finisher0061",2),
                ("attacks/grenade_death/tax_man/grenade_finisher0063",1)),

            ["seated_swirl"] = C(Frames(s, "seated/wine_swirl/wine_swirl_00",
                "seated/wine_swirl/wine_swirl_01"), 3, true, -319, -293, true),
            ["seated_drink"] = CSS(s, 24, false, -319, -293, true,
                new[] { (0f,-319f,-293f), (.375f,-318f,-297f) },
                ("seated/wine_drink/wine_drink_00",9), ("seated/wine_drink/wine_drink_01",30)),
            ["seated_reveal"] = CSS(s, 24, false, -319, -293, true,
                new[] { (0f,-319f,-293f), (1.375f,-318f,-297f) },
                ("seated/wine_swirl/wine_swirl_00",12), ("seated/wine_swirl/wine_swirl_01",12),
                ("seated/wine_drink/wine_drink_00",9), ("seated/wine_drink/wine_drink_01",30)),
            ["seated_laugh"] = CSS(s, 24, false, -318, -293, true, Array.Empty<(float,float,float)>(),
                ("seated/laughing/laughter_00",5), ("seated/laughing/laughter_01",5),
                ("seated/laughing/laughter_02",2), ("seated/laughing/laughter_01",5),
                ("seated/laughing/laughter_02",2), ("seated/laughing/laughter_01",5),
                ("seated/laughing/laughter_02",2), ("seated/laughing/laughter_01",5),
                ("seated/laughing/laughter_02",3), ("seated/laughing/laughter_01",5),
                ("seated/laughing/laughter_02",3), ("seated/laughing/laughter_01",5),
                ("seated/laughing/laughter_02",2), ("seated/laughing/laughter_01",5),
                ("seated/laughing/laughter_02",2), ("seated/laughing/laughter_01",5),
                ("seated/laughing/laughter_02",2), ("seated/laughing/laughter_01",5),
                ("seated/laughing/laughter_02",5), ("seated/laughing/laughter_03",19)),
            ["seated_engage"] = CSS(s, 24, false, -318, -306, true,
                new[] { (0f,-318f,-306f), (1.20833f,-221f,-306f) },
                ("seated/engage/engage_00",30), ("seated/engage/engage_01",6),
                ("seated/engage/engage_02",6)),

            ["vfx_area_back"] = C(FolderFrames(s, "attacks/area_attack/back_lightning"), 24, false, 30, -688),
            ["vfx_area_front"] = C(FolderFrames(s, "attacks/area_attack/front_lightning"), 24, false, -77, -511),
            ["vfx_area_ground"] = C(FolderFrames(s, "attacks/area_attack/ground_lightning"), 5, true, -30, 159),
            ["vfx_area_explosion"] = C(FolderFrames(s, "attacks/area_attack/explosion"), 5, true, -159, -414),
            ["vfx_area_smoke"] = C(FolderFrames(s, "attacks/area_attack/smoke"), 5, false, -758, -747),
            ["vfx_death_back"] = C(FolderFrames(s, "attacks/grenade_death/back_ligthning"), 5, true, -82, -720),
            ["vfx_death_front"] = C(FolderFrames(s, "attacks/grenade_death/front_lightning"), 5, true, -77, -551),
            ["vfx_death_ground"] = C(FolderFrames(s, "attacks/grenade_death/ground_lightning"), 5, true, -47, 149),
            ["vfx_death_explosion"] = C(FolderFrames(s, "attacks/grenade_death/explosion"), 24, false, -135, -486),
            ["vfx_death_smoke_v"] = C(FolderFrames(s, "attacks/grenade_death/smoke"), 24, false, -758, -747),
            ["vfx_death_smoke_h"] = C(FolderFrames(s, "attacks/grenade_death/smoke_horizontal"), 5, true, -62, -211),
            ["vfx_coins"] = C(FolderFrames(s, "attacks/grenade_death/coins"), 5, true, -62, -211),
        };
    }
}
