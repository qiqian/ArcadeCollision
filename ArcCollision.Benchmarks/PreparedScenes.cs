using Ref = ArcCollision.Ref;
using Wrapper = ArcCollision.Wrapper;

namespace ArcCollision.Benchmarks;

internal sealed class RefPreparedScene
{
    public Ref.Shape[] StaticShapes { get; }
    public Ref.Shape[] DynamicInitialShapes { get; }
    public Ref.Shape[] DynamicFrameShapes { get; }
    public Ref.Transform[] DynamicFrameTransforms { get; }
    private readonly Ref.Polygon[] _polygons;

    public RefPreparedScene(BenchmarkScenario source)
    {
        _polygons = CreatePolygons();
        StaticShapes = Convert(source.StaticShapes);
        DynamicInitialShapes = Convert(source.DynamicInitialShapes);
        DynamicFrameShapes = Convert(source.DynamicFrameShapes);
        DynamicFrameTransforms = ConvertTransforms(
            source.DynamicFrameShapes, source.DynamicInitialShapes);
    }

    private Ref.Shape[] Convert(ShapeSpec[] source)
    {
        var result = new Ref.Shape[source.Length];
        for (int i = 0; i < result.Length; i++)
            result[i] = ToShape(source[i]);
        return result;
    }

    private Ref.Shape ToShape(ShapeSpec spec)
    {
        var center = new Ref.Vec2(ToFloat(spec.X), ToFloat(spec.Y));
        return spec.Kind switch
        {
            BenchmarkShapeKind.Circle => new Ref.Circle(center, ToFloat(spec.Radius)),
            BenchmarkShapeKind.Aabb => new Ref.Aabb(
                center, new Ref.Vec2(ToFloat(spec.SizeX), ToFloat(spec.SizeY))),
            BenchmarkShapeKind.Capsule => new Ref.Capsule(
                new Ref.Vec2(ToFloat(spec.X - spec.SizeX), ToFloat(spec.Y - spec.SizeY)),
                new Ref.Vec2(ToFloat(spec.X + spec.SizeX), ToFloat(spec.Y + spec.SizeY)),
                ToFloat(spec.Radius)),
            BenchmarkShapeKind.Obb => new Ref.Obb(
                center, new Ref.Vec2(ToFloat(spec.SizeX), ToFloat(spec.SizeY)),
                new Ref.Angle32(spec.Angle)),
            BenchmarkShapeKind.Polygon => new Ref.Shape(
                _polygons[spec.PolygonVariant], center, new Ref.Angle32(spec.Angle)),
            _ => throw new InvalidOperationException(),
        };
    }

    private static Ref.Transform[] ConvertTransforms(
        ShapeSpec[] frames, ShapeSpec[] initial)
    {
        var result = new Ref.Transform[frames.Length];
        for (int i = 0; i < result.Length; i++)
        {
            ShapeSpec frame = frames[i];
            ShapeSpec basis = initial[i % initial.Length];
            uint rotation = frame.Kind is BenchmarkShapeKind.Obb or BenchmarkShapeKind.Polygon
                ? unchecked(frame.Angle - basis.Angle)
                : 0u;
            result[i] = new Ref.Transform(
                new Ref.Vec2(ToFloat(frame.X), ToFloat(frame.Y)),
                new Ref.Angle32(rotation), 1f);
        }
        return result;
    }

    private static Ref.Polygon[] CreatePolygons()
    {
        var result = new Ref.Polygon[PolygonTemplates.Count];
        for (int i = 0; i < result.Length; i++)
        {
            (int X, int Y)[] source = PolygonTemplates.RawVertices[i];
            var vertices = new Ref.Vec2[source.Length];
            for (int j = 0; j < vertices.Length; j++)
                vertices[j] = new Ref.Vec2(ToFloat(source[j].X), ToFloat(source[j].Y));
            result[i] = new Ref.Polygon(vertices);
        }
        return result;
    }

    private static float ToFloat(int raw) => raw * (1f / 256f);
}

internal sealed class WrapperPreparedScene
{
    public Wrapper.Shape[] StaticShapes { get; }
    public Wrapper.Shape[] DynamicInitialShapes { get; }
    public Wrapper.Shape[] DynamicFrameShapes { get; }
    public Wrapper.Transform[] DynamicFrameTransforms { get; }
    private readonly Wrapper.Polygon[] _polygons;

    public WrapperPreparedScene(BenchmarkScenario source)
    {
        _polygons = CreatePolygons();
        StaticShapes = Convert(source.StaticShapes);
        DynamicInitialShapes = Convert(source.DynamicInitialShapes);
        DynamicFrameShapes = Convert(source.DynamicFrameShapes);
        DynamicFrameTransforms = ConvertTransforms(
            source.DynamicFrameShapes, source.DynamicInitialShapes);
    }

    private Wrapper.Shape[] Convert(ShapeSpec[] source)
    {
        var result = new Wrapper.Shape[source.Length];
        for (int i = 0; i < result.Length; i++)
            result[i] = ToShape(source[i]);
        return result;
    }

    private Wrapper.Shape ToShape(ShapeSpec spec)
    {
        var center = new Wrapper.Vec2(ToFloat(spec.X), ToFloat(spec.Y));
        return spec.Kind switch
        {
            BenchmarkShapeKind.Circle => new Wrapper.Circle(center, ToFloat(spec.Radius)),
            BenchmarkShapeKind.Aabb => new Wrapper.Aabb(
                center, new Wrapper.Vec2(ToFloat(spec.SizeX), ToFloat(spec.SizeY))),
            BenchmarkShapeKind.Capsule => new Wrapper.Capsule(
                new Wrapper.Vec2(ToFloat(spec.X - spec.SizeX), ToFloat(spec.Y - spec.SizeY)),
                new Wrapper.Vec2(ToFloat(spec.X + spec.SizeX), ToFloat(spec.Y + spec.SizeY)),
                ToFloat(spec.Radius)),
            BenchmarkShapeKind.Obb => new Wrapper.Obb(
                center, new Wrapper.Vec2(ToFloat(spec.SizeX), ToFloat(spec.SizeY)),
                new Wrapper.Angle32(spec.Angle)),
            BenchmarkShapeKind.Polygon => new Wrapper.Shape(
                _polygons[spec.PolygonVariant], center, new Wrapper.Angle32(spec.Angle)),
            _ => throw new InvalidOperationException(),
        };
    }

    private static Wrapper.Transform[] ConvertTransforms(
        ShapeSpec[] frames, ShapeSpec[] initial)
    {
        var result = new Wrapper.Transform[frames.Length];
        for (int i = 0; i < result.Length; i++)
        {
            ShapeSpec frame = frames[i];
            ShapeSpec basis = initial[i % initial.Length];
            uint rotation = frame.Kind is BenchmarkShapeKind.Obb or BenchmarkShapeKind.Polygon
                ? unchecked(frame.Angle - basis.Angle)
                : 0u;
            result[i] = new Wrapper.Transform(
                new Wrapper.Vec2(ToFloat(frame.X), ToFloat(frame.Y)),
                new Wrapper.Angle32(rotation), 1f);
        }
        return result;
    }

    private static Wrapper.Polygon[] CreatePolygons()
    {
        var result = new Wrapper.Polygon[PolygonTemplates.Count];
        for (int i = 0; i < result.Length; i++)
        {
            (int X, int Y)[] source = PolygonTemplates.RawVertices[i];
            var vertices = new Wrapper.Vec2[source.Length];
            for (int j = 0; j < vertices.Length; j++)
                vertices[j] = new Wrapper.Vec2(ToFloat(source[j].X), ToFloat(source[j].Y));
            result[i] = new Wrapper.Polygon(vertices);
        }
        return result;
    }

    private static float ToFloat(int raw) => raw * (1f / 256f);
}
