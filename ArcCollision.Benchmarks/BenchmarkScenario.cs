namespace ArcCollision.Benchmarks;

internal enum BenchmarkShapeKind
{
    Circle,
    Aabb,
    Capsule,
    Obb,
    Polygon,
}

// All positions and sizes are generated as raw 24.8 integers. Converting them
// with /256f is exact, so both backends receive byte-identical public inputs.
internal readonly record struct ShapeSpec(
    BenchmarkShapeKind Kind,
    int X,
    int Y,
    int SizeX,
    int SizeY,
    int Radius,
    uint Angle,
    int PolygonVariant)
{
    public ShapeSpec At(int x, int y, uint angle) => this with
    {
        X = x,
        Y = y,
        Angle = angle,
    };
}

internal readonly record struct DynamicSpec(
    ShapeSpec BaseShape,
    int PhaseX,
    int PhaseY,
    int SpeedX,
    int SpeedY,
    int Travel,
    uint AngularVelocity);

internal sealed class BenchmarkScenario
{
    public ShapeSpec[] StaticShapes { get; }
    public ShapeSpec[] DynamicInitialShapes { get; }
    public ShapeSpec[] DynamicFrameShapes { get; }
    public int FrameCount { get; }
    public int DynamicCount => DynamicInitialShapes.Length;

    private BenchmarkScenario(
        ShapeSpec[] staticShapes,
        ShapeSpec[] dynamicInitialShapes,
        ShapeSpec[] dynamicFrameShapes,
        int frameCount)
    {
        StaticShapes = staticShapes;
        DynamicInitialShapes = dynamicInitialShapes;
        DynamicFrameShapes = dynamicFrameShapes;
        FrameCount = frameCount;
    }

    public static BenchmarkScenario Create(BenchmarkOptions options)
    {
        var random = new StableRandom(options.Seed);
        int total = checked(options.StaticCount + options.DynamicCount);
        int clusterCount = Math.Clamp(total / 64, 8, 64);
        var clusters = new (int X, int Y)[clusterCount];
        for (int i = 0; i < clusters.Length; i++)
        {
            clusters[i] = (
                random.NextInt(-5000 * 256, 5000 * 256 + 1),
                random.NextInt(-5000 * 256, 5000 * 256 + 1));
        }

        ShapeSpec NextShape()
        {
            (int clusterX, int clusterY) = clusters[random.NextInt(clusters.Length)];
            int x = clusterX + random.NextInt(-48 * 256, 48 * 256 + 1);
            int y = clusterY + random.NextInt(-48 * 256, 48 * 256 + 1);
            int selector = random.NextInt(100);
            BenchmarkShapeKind kind = selector switch
            {
                < 25 => BenchmarkShapeKind.Circle,
                < 50 => BenchmarkShapeKind.Aabb,
                < 70 => BenchmarkShapeKind.Capsule,
                < 90 => BenchmarkShapeKind.Obb,
                _ => BenchmarkShapeKind.Polygon,
            };
            int sizeX = random.NextInt(3 * 256, 15 * 256 + 1);
            int sizeY = random.NextInt(3 * 256, 15 * 256 + 1);
            int radius = random.NextInt(2 * 256, 8 * 256 + 1);
            return new ShapeSpec(
                kind, x, y, sizeX, sizeY, radius,
                random.NextUInt32(), random.NextInt(PolygonTemplates.Count));
        }

        var staticShapes = new ShapeSpec[options.StaticCount];
        for (int i = 0; i < staticShapes.Length; i++)
            staticShapes[i] = NextShape();

        var dynamicSpecs = new DynamicSpec[options.DynamicCount];
        var dynamicInitial = new ShapeSpec[options.DynamicCount];
        for (int i = 0; i < dynamicSpecs.Length; i++)
        {
            ShapeSpec shape = NextShape();
            int signedAngularStep = random.NextInt(1, 9) * 0x00100000;
            if ((random.NextUInt32() & 1) != 0) signedAngularStep = -signedAngularStep;
            dynamicSpecs[i] = new DynamicSpec(
                shape,
                random.NextInt(256),
                random.NextInt(256),
                random.NextSignedSpeed(),
                random.NextSignedSpeed(),
                random.NextInt(4 * 256, 14 * 256 + 1),
                unchecked((uint)signedAngularStep));
            dynamicInitial[i] = shape;
        }

        var dynamicFrames = new ShapeSpec[checked(options.Frames * options.DynamicCount)];
        for (int frame = 0; frame < options.Frames; frame++)
        {
            int frameNumber = frame + 1;
            int offset = frame * options.DynamicCount;
            for (int i = 0; i < dynamicSpecs.Length; i++)
            {
                DynamicSpec dynamic = dynamicSpecs[i];
                int x = dynamic.BaseShape.X + TriangleOffset(
                    dynamic.PhaseX + frameNumber * dynamic.SpeedX, dynamic.Travel);
                int y = dynamic.BaseShape.Y + TriangleOffset(
                    dynamic.PhaseY + frameNumber * dynamic.SpeedY, dynamic.Travel);
                uint angle = unchecked(dynamic.BaseShape.Angle
                    + (uint)frameNumber * dynamic.AngularVelocity);
                dynamicFrames[offset + i] = dynamic.BaseShape.At(x, y, angle);
            }
        }

        return new BenchmarkScenario(
            staticShapes, dynamicInitial, dynamicFrames, options.Frames);
    }

    private static int TriangleOffset(int phase, int travel)
    {
        int position = phase & 255;
        int wave = position < 128 ? position * 2 - 127 : 383 - position * 2;
        return wave * travel / 127;
    }
}

internal static class PolygonTemplates
{
    public static int Count => RawVertices.Length;

    public static readonly (int X, int Y)[][] RawVertices =
    [
        [(-8 * 256, -5 * 256), (0, -9 * 256), (8 * 256, -5 * 256),
         (9 * 256, 3 * 256), (0, 8 * 256), (-9 * 256, 3 * 256)],
        [(-10 * 256, -8 * 256), (10 * 256, -8 * 256), (10 * 256, -2 * 256),
         (2 * 256, -2 * 256), (2 * 256, 8 * 256), (-10 * 256, 8 * 256)],
        [(-7 * 256, -6 * 256), (8 * 256, -4 * 256), (1 * 256, 9 * 256)],
    ];
}

// XorShift64* has a fully specified sequence, unlike System.Random whose
// implementation may change between runtime versions.
internal struct StableRandom
{
    private ulong _state;

    public StableRandom(ulong seed)
    {
        _state = seed != 0 ? seed : 0x9E3779B97F4A7C15UL;
    }

    public uint NextUInt32()
    {
        ulong value = _state;
        value ^= value >> 12;
        value ^= value << 25;
        value ^= value >> 27;
        _state = value;
        return (uint)((value * 2685821657736338717UL) >> 32);
    }

    public int NextInt(int exclusiveMax)
    {
        if (exclusiveMax <= 0) throw new ArgumentOutOfRangeException(nameof(exclusiveMax));
        return (int)(((ulong)NextUInt32() * (uint)exclusiveMax) >> 32);
    }

    public int NextInt(int inclusiveMin, int exclusiveMax)
    {
        if (exclusiveMax <= inclusiveMin) throw new ArgumentOutOfRangeException(nameof(exclusiveMax));
        return inclusiveMin + NextInt(exclusiveMax - inclusiveMin);
    }

    public int NextSignedSpeed()
    {
        int speed = NextInt(1, 6);
        return (NextUInt32() & 1) == 0 ? speed : -speed;
    }
}
