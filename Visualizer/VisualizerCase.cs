using System.Text.Json;
using System.Text.Json.Serialization;

namespace ArcCollision.Visualizer;

/// <summary>
/// Versioned, backend-independent snapshot of one visualizer test. Handle
/// positions are the actual collision inputs, so JSON float round-tripping can
/// reproduce a dragged failure without depending on the window's later size.
/// </summary>
internal sealed class VisualizerCase
{
    public const int CurrentVersion = 1;

    public int Version { get; set; } = CurrentVersion;
    public DateTimeOffset SavedAtUtc { get; set; } = DateTimeOffset.UtcNow;
    public bool GenericSweep { get; set; }
    public SceneKind Scene { get; set; }
    public ShapeKind SweepMover { get; set; }
    public ShapeKind SweepTarget { get; set; }
    public bool ShowContacts { get; set; }
    public int CanvasWidth { get; set; }
    public int CanvasHeight { get; set; }
    public string ResultText { get; set; } = string.Empty;
    public List<VisualizerHandleState>? Handles { get; set; } = new();
}

internal sealed class VisualizerHandleState
{
    public string Name { get; set; } = string.Empty;
    public float X { get; set; }
    public float Y { get; set; }
}

internal static class VisualizerCaseSerializer
{
    private static readonly JsonSerializerOptions Options = CreateOptions();

    public static void Save(string path, VisualizerCase testCase)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(path);
        ArgumentNullException.ThrowIfNull(testCase);
        File.WriteAllText(path, JsonSerializer.Serialize(testCase, Options));
    }

    public static VisualizerCase Load(string path)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(path);
        VisualizerCase? testCase = JsonSerializer.Deserialize<VisualizerCase>(
            File.ReadAllText(path), Options);
        if (testCase is null)
            throw new InvalidDataException("The file does not contain a visualizer test case.");
        if (testCase.Version != VisualizerCase.CurrentVersion)
        {
            throw new InvalidDataException(
                $"Unsupported visualizer test version {testCase.Version}; " +
                $"expected {VisualizerCase.CurrentVersion}.");
        }
        if (testCase.Handles is null || testCase.Handles.Count == 0)
            throw new InvalidDataException("The visualizer test has no handles.");
        return testCase;
    }

    private static JsonSerializerOptions CreateOptions()
    {
        var options = new JsonSerializerOptions
        {
            WriteIndented = true,
            PropertyNameCaseInsensitive = true,
        };
        options.Converters.Add(new JsonStringEnumConverter(allowIntegerValues: false));
        return options;
    }
}
