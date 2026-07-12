using System.Reflection;
using System.Runtime.InteropServices;
using Ref = ArcCollision;
using Native = ArcCollision.Wrapper;
using Xunit;

namespace ArcCollision.Tests;

[Collection("ArcWorld lifecycle")]
public class WrapperParityTests
{
    // Both backends run the identical all-integer Q1.30 core and build every
    // rotation axis from the same integer Angle32 -> CORDIC path, so they are
    // designed to agree to within the float->fixed boundary rounding (<=~1 grid
    // cell). These tolerances enforce that: they are ~20-40x tighter than "shapes
    // roughly agree", so a native regression (e.g. axis dropped back to 24.8)
    // that reintroduces an extent-relative error would break the build.
    private const float Grid = 1f / 256f;

    private static float PairDepthTol(Ref.Shape a, Ref.Shape b)
    {
        float reach = MathF.Abs(a.Bounds.HalfExtents.X) + MathF.Abs(a.Bounds.HalfExtents.Y)
            + MathF.Abs(b.Bounds.HalfExtents.X) + MathF.Abs(b.Bounds.HalfExtents.Y);
        // 3 grid cells of boundary rounding + a whisper of size scaling. Note the
        // size term is ~5000x smaller than the 0.4%-of-extent error a 24.8 axis
        // regression would produce, so this stays sensitive to such a regression.
        return 3f * Grid + reach * 2e-4f;
    }
    [Fact]
    public void WrapperExposesSameDeclaredPublicSurfaceAsReference()
    {
        Assembly reference = typeof(Ref.ArcWorld).Assembly;
        Assembly wrapper = typeof(Native.ArcWorld).Assembly;
        string[] referenceTypes = PublicTypes(reference).Select(TypeKey).Order().ToArray();
        string[] wrapperTypes = PublicTypes(wrapper).Select(TypeKey).Order().ToArray();
        Assert.Equal(referenceTypes, wrapperTypes);

        var differences = new List<string>();
        foreach (Type referenceType in PublicTypes(reference))
        {
            Type wrapperType = wrapper.GetType(
                referenceType.FullName!.Replace("ArcCollision", "ArcCollision.Wrapper", StringComparison.Ordinal),
                throwOnError: true)!;
            if (TypeShape(referenceType) != TypeShape(wrapperType))
                differences.Add($"{referenceType.Name} type\nExpected: {TypeShape(referenceType)}"
                    + $"\nActual: {TypeShape(wrapperType)}");
            string[] expectedMembers = PublicMembers(referenceType);
            string[] actualMembers = PublicMembers(wrapperType);
            if (!expectedMembers.SequenceEqual(actualMembers))
                differences.Add($"{referenceType.Name}\nExpected only:\n"
                    + $"{string.Join('\n', expectedMembers.Except(actualMembers))}"
                    + $"\nActual only:\n{string.Join('\n', actualMembers.Except(expectedMembers))}");
        }
        Assert.True(differences.Count == 0, string.Join("\n\n", differences));
    }

    [Fact]
    public void PrimitiveNarrowphaseMatchesReference()
    {
        Ref.Circle refCircle = new(new Ref.Vec2(0, 0), 2);
        Ref.Aabb refBox = new(new Ref.Vec2(2.5f, 0), new Ref.Vec2(1, 1));
        Native.Circle nativeCircle = new(new Native.Vec2(0, 0), 2);
        Native.Aabb nativeBox = new(new Native.Vec2(2.5f, 0), new Native.Vec2(1, 1));

        Ref.Manifold expected = Ref.Collide.CircleVsAabb(refCircle, refBox);
        Native.Manifold actual = Native.Collide.CircleVsAabb(nativeCircle, nativeBox);

        // Clean inputs (exact multiples of 1/256, no rotation) => the integer
        // cores are bit-identical up to boundary rounding: hold them to 1 cell.
        Assert.Equal(expected.Colliding, actual.Colliding);
        Assert.Equal(expected.Depth, actual.Depth, Grid);
        Assert.Equal(expected.Normal.X, actual.Normal.X, Grid);
        Assert.Equal(expected.Normal.Y, actual.Normal.Y, Grid);
    }

    [Fact]
    public void PolygonAndGenericSweepRunNatively()
    {
        var polygon = new Native.Polygon(
            new Native.Vec2(-1, -1), new Native.Vec2(1, -1),
            new Native.Vec2(1, 1), new Native.Vec2(-1, 1));
        Native.Shape target = new Native.Shape(
            polygon, new Native.Vec2(5, 0), new Native.Angle32(0));
        Native.Shape mover = new Native.Circle(Native.Vec2.Zero, .5f);

        Native.SweepHit hit = Native.Sweep.MovingShapeVsShape(
            mover, new Native.Vec2(10, 0), target);

        Assert.True(hit.Hit);
        Assert.InRange(hit.Time, .34f, .36f);
    }

    [Fact]
    public void NativeWorldSupportsFiltersQueriesContactsAndCasts()
    {
        const uint attack = 1u << 1, hurt = 1u << 2;
        using var world = new Native.ArcWorld();
        Native.ArcHandle target = world.AddStatic(10,
            new Native.Circle(new Native.Vec2(4, 0), 1),
            new Native.CollisionFilter(hurt, attack));
        Native.Shape query = new Native.Circle(Native.Vec2.Zero, .5f);
        var filter = new Native.CollisionFilter(attack, hurt);
        var handles = new List<Native.ArcHandle>();

        world.Query(new Native.Circle(new Native.Vec2(4, 0), 2), filter, handles);
        Assert.Equal(target, Assert.Single(handles));
        Assert.True(world.ShapeCast(query, new Native.Vec2(10, 0), filter, out Native.WorldCastHit hit));
        Assert.Equal(10, hit.Handle.EntityId);

        world.SetEnabled(target, false);
        Assert.False(world.ShapeCast(query, new Native.Vec2(10, 0), filter, out _));
        Assert.True(world.IsValid(target));
    }

    [Fact]
    public void EveryOrderedShapePairMatchesReferenceCollisionState()
    {
        (Ref.Shape Ref, Native.Shape Native)[] shapes = CreateShapes();
        foreach (var a in shapes)
        foreach (var b in shapes)
        {
            Ref.Manifold expected = Ref.Collide.ShapeVsShape(a.Ref, b.Ref);
            Native.Manifold actual = Native.Collide.ShapeVsShape(a.Native, b.Native);
            Assert.True(expected.Colliding == actual.Colliding,
                $"{a.Ref.Kind} vs {b.Ref.Kind}: ref={expected.Colliding}, native={actual.Colliding}");
            if (!expected.Colliding) continue;
            Assert.InRange(MathF.Abs(expected.Depth - actual.Depth), 0, PairDepthTol(a.Ref, b.Ref));
            Ref.Vec2 centerDelta = b.Ref.Bounds.Center - a.Ref.Bounds.Center;
            // Skip the direction check on grazing contacts (depth ~0), where the
            // normal is genuinely ill-conditioned in both backends; otherwise the
            // Q1.30 axes must line up to within ~8 cells of angular error.
            if (centerDelta.LengthSquared > .01f && expected.Depth > 3f * Grid)
                Assert.True(expected.Normal.X * actual.Normal.X + expected.Normal.Y * actual.Normal.Y > 1f - 8f * Grid,
                    $"{a.Ref.Kind} vs {b.Ref.Kind}: ref=({expected.Normal.X},{expected.Normal.Y}) native=({actual.Normal.X},{actual.Normal.Y}) depth={expected.Depth}/{actual.Depth}");
        }
    }

    [Fact]
    public void EveryOrderedShapeSweepHitsInBothImplementations()
    {
        (Ref.Shape Ref, Native.Shape Native)[] targets = CreateShapes();
        foreach (var mover in targets)
        foreach (var target in targets)
        {
            Ref.Shape refMover = mover.Ref.Moved(new Ref.Vec2(-8, 0));
            Native.Shape nativeMover = mover.Native.Moved(new Native.Vec2(-8, 0));
            Ref.SweepHit expected = Ref.Sweep.MovingShapeVsShape(
                refMover, new Ref.Vec2(16, 0), target.Ref);
            Native.SweepHit actual = Native.Sweep.MovingShapeVsShape(
                nativeMover, new Native.Vec2(16, 0), target.Native);
            Assert.Equal(expected.Hit, actual.Hit);
            // 16-unit motion; position parity ~1/256 => TOI parity ~(1/256)/16.
            // Kept at 0.01 (8x tighter than before) to absorb the ill-conditioned
            // grazing-TOI regime while still enforcing close agreement.
            if (expected.Hit)
                Assert.InRange(MathF.Abs(expected.Time - actual.Time), 0, .01f);
        }
    }

    [Fact]
    public void LargeFarRotatedBoxes_AgreeTightly_ExposeAxisPrecisionRegression()
    {
        // The near-origin unit shapes above cannot distinguish a Q1.30 axis from a
        // 24.8 one: the 0.4%-of-extent error is only ~0.008 on a 1-unit box. Here
        // the boxes are large and far from the origin, so a native axis regression
        // to 24.8 would put depth off by ~extent*0.4% (>1 unit on a 300-unit box)
        // and fail, while two honest integer Q1.30 cores still agree to ~1 cell.
        (float scale, Ref.Vec2 rOff, Native.Vec2 nOff)[] regimes =
        {
            (40f,  new Ref.Vec2(0, 0),               new Native.Vec2(0, 0)),
            (300f, new Ref.Vec2(400_000, -250_000),  new Native.Vec2(400_000, -250_000)),
            (300f, new Ref.Vec2(-1_500_000, 900_000), new Native.Vec2(-1_500_000, 900_000)),
        };
        (float angA, float angB)[] rots = { (.3f, -.4f), (1.1f, .2f), (-.9f, 2.4f) };

        foreach (var (scale, rOff, nOff) in regimes)
        foreach (var (angA, angB) in rots)
        {
            var half = (x: scale, y: scale * .7f);
            // Centres separated by less than the summed reach => deep overlap.
            var sep = scale * .6f;

            var refA = new Ref.Obb(rOff, new Ref.Vec2(half.x, half.y), angA);
            var refB = new Ref.Obb(new Ref.Vec2(rOff.X + sep, rOff.Y), new Ref.Vec2(half.x, half.y), angB);
            var natA = new Native.Obb(nOff, new Native.Vec2(half.x, half.y), angA);
            var natB = new Native.Obb(new Native.Vec2(nOff.X + sep, nOff.Y), new Native.Vec2(half.x, half.y), angB);

            Ref.Manifold expected = Ref.Collide.ShapeVsShape(new Ref.Shape(refA), new Ref.Shape(refB));
            Native.Manifold actual = Native.Collide.ShapeVsShape(new Native.Shape(natA), new Native.Shape(natB));

            string repro = $"scale={scale} off=({rOff.X},{rOff.Y}) ang=({angA},{angB})";
            Assert.True(expected.Colliding && actual.Colliding, $"expected deep overlap: {repro}");

            float depthTol = 3f * Grid + scale * 8e-4f;   // << the ~scale*4e-3 a 24.8 axis would cost
            Assert.InRange(MathF.Abs(expected.Depth - actual.Depth), 0, depthTol);
            Assert.True(expected.Normal.X * actual.Normal.X + expected.Normal.Y * actual.Normal.Y > 1f - 8f * Grid,
                $"normal diverged: ref=({expected.Normal.X},{expected.Normal.Y}) native=({actual.Normal.X},{actual.Normal.Y}): {repro}");
        }
    }

    [Fact]
    public void RandomPrimitiveCollisionStatesMatchReference()
    {
        var random = new Random(81723);
        for (int i = 0; i < 1000; i++)
        {
            var a = RandomShape(random, random.Next(5));
            var b = RandomShape(random, random.Next(5));
            bool expected = Ref.Collide.Overlaps(a.Ref, b.Ref);
            bool actual = Native.Collide.Overlaps(a.Native, b.Native);
            Assert.True(expected == actual,
                $"case {i}: {a.Ref.Kind} vs {b.Ref.Kind}, ref={expected}, native={actual}");
        }
    }

    [Fact]
    public void NativeWorldPairAndQuerySetsMatchReferenceThroughMutations()
    {
        using var reference = new Ref.ArcWorld(4);
        using var native = new Native.ArcWorld(4);
        var random = new Random(5519);
        var refHandles = new List<Ref.ArcHandle>();
        var nativeHandles = new List<Native.ArcHandle>();
        for (int i = 0; i < 80; i++)
        {
            var shape = RandomShape(random, random.Next(5));
            bool isStatic = i % 4 == 0;
            refHandles.Add(isStatic ? reference.AddStatic(i, shape.Ref) : reference.Add(i, shape.Ref));
            nativeHandles.Add(isStatic ? native.AddStatic(i, shape.Native) : native.Add(i, shape.Native));
        }

        for (int i = 0; i < 10; i++)
        {
            reference.SetEnabled(refHandles[i * 3], false);
            native.SetEnabled(nativeHandles[i * 3], false);
        }
        reference.BuildStatic(); native.BuildStatic();

        var refPairs = new List<Ref.CandidatePair>();
        var nativePairs = new List<Native.CandidatePair>();
        reference.ComputePairs(refPairs); native.ComputePairs(nativePairs);
        Assert.Equal(
            refPairs.Select(p => (p.A.EntityId, p.B.EntityId)).Order(),
            nativePairs.Select(p => (p.A.EntityId, p.B.EntityId)).Order());

        var refQuery = new List<Ref.ArcHandle>();
        var nativeQuery = new List<Native.ArcHandle>();
        reference.Query(new Ref.Aabb(Ref.Vec2.Zero, new Ref.Vec2(4, 4)), refQuery);
        native.Query(new Native.Aabb(Native.Vec2.Zero, new Native.Vec2(4, 4)), nativeQuery);
        Assert.Equal(refQuery.Select(h => h.EntityId), nativeQuery.Select(h => h.EntityId));
    }

    private static (Ref.Shape Ref, Native.Shape Native)[] CreateShapes()
    {
        var refPolygon = new Ref.Polygon(
            new Ref.Vec2(-1.2f, -.8f), new Ref.Vec2(1.2f, -.8f),
            new Ref.Vec2(1, 1), new Ref.Vec2(-1, 1));
        var nativePolygon = new Native.Polygon(
            new Native.Vec2(-1.2f, -.8f), new Native.Vec2(1.2f, -.8f),
            new Native.Vec2(1, 1), new Native.Vec2(-1, 1));
        return
        [
            (new Ref.Circle(new Ref.Vec2(0, 0), 1.1f), new Native.Circle(new Native.Vec2(0, 0), 1.1f)),
            (new Ref.Aabb(new Ref.Vec2(.2f, 0), new Ref.Vec2(1, .9f)), new Native.Aabb(new Native.Vec2(.2f, 0), new Native.Vec2(1, .9f))),
            (new Ref.Capsule(new Ref.Vec2(-1, 0), new Ref.Vec2(1, 0), .65f), new Native.Capsule(new Native.Vec2(-1, 0), new Native.Vec2(1, 0), .65f)),
            (new Ref.Obb(new Ref.Vec2(0, 0), new Ref.Vec2(1, .7f), .3f), new Native.Obb(new Native.Vec2(0, 0), new Native.Vec2(1, .7f), .3f)),
            (new Ref.Shape(refPolygon), new Native.Shape(nativePolygon)),
        ];
    }

    private static (Ref.Shape Ref, Native.Shape Native) RandomShape(Random random, int kind)
    {
        float F(float min, float max) => min + (float)random.NextDouble() * (max - min);
        float x = F(-6, 6), y = F(-6, 6), sx = F(.2f, 2), sy = F(.2f, 2);
        return kind switch
        {
            0 => (new Ref.Circle(new Ref.Vec2(x, y), sx),
                new Native.Circle(new Native.Vec2(x, y), sx)),
            1 => (new Ref.Aabb(new Ref.Vec2(x, y), new Ref.Vec2(sx, sy)),
                new Native.Aabb(new Native.Vec2(x, y), new Native.Vec2(sx, sy))),
            2 => (new Ref.Capsule(new Ref.Vec2(x - sx, y), new Ref.Vec2(x + sx, y + sy * .3f), sy * .5f),
                new Native.Capsule(new Native.Vec2(x - sx, y), new Native.Vec2(x + sx, y + sy * .3f), sy * .5f)),
            3 => ObbPair(x, y, sx, sy, F(-2, 2)),
            _ => PolygonPair(x, y, sx, sy),
        };

        static (Ref.Shape, Native.Shape) ObbPair(float x, float y, float sx, float sy, float angle) =>
            (new Ref.Obb(new Ref.Vec2(x, y), new Ref.Vec2(sx, sy), angle),
                new Native.Obb(new Native.Vec2(x, y), new Native.Vec2(sx, sy), angle));

        static (Ref.Shape, Native.Shape) PolygonPair(float x, float y, float sx, float sy)
        {
            var rp = new Ref.Polygon(
                new Ref.Vec2(-sx, -sy), new Ref.Vec2(sx, -sy),
                new Ref.Vec2(sx * .8f, sy), new Ref.Vec2(-sx * .8f, sy));
            var np = new Native.Polygon(
                new Native.Vec2(-sx, -sy), new Native.Vec2(sx, -sy),
                new Native.Vec2(sx * .8f, sy), new Native.Vec2(-sx * .8f, sy));
            return (new Ref.Shape(rp, new Ref.Vec2(x, y), new Ref.Angle32(0)),
                new Native.Shape(np, new Native.Vec2(x, y), new Native.Angle32(0)));
        }
    }

    private static IEnumerable<Type> PublicTypes(Assembly assembly) =>
        assembly.GetExportedTypes().Where(t => t.Namespace is "ArcCollision" or "ArcCollision.Wrapper");

    private static string TypeKey(Type type) => Normalize(type.FullName!);

    private static string[] PublicMembers(Type type) => type
        .GetMembers(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly)
        .Where(m => m.MemberType is MemberTypes.Constructor or MemberTypes.Method
            or MemberTypes.Property or MemberTypes.Field)
        .Select(MemberKey)
        .Order()
        .ToArray();

    private static string MemberKey(MemberInfo member) => member switch
    {
        MethodBase method => $"{member.MemberType}:{method.Name}"
            + $"({string.Join(',', method.GetParameters().Select(ParameterKey))})"
            + $":static={method.IsStatic}:generic="
            + (method is MethodInfo genericMethod ? genericMethod.GetGenericArguments().Length : 0)
            + (method is MethodInfo info ? $":{Normalize(info.ReturnType.ToString())}" : ""),
        PropertyInfo property => $"Property:{property.Name}:{Normalize(property.PropertyType.ToString())}"
            + $":index=({string.Join(',', property.GetIndexParameters().Select(ParameterKey))})",
        FieldInfo field => $"Field:{field.Name}:{Normalize(field.FieldType.ToString())}"
            + $":static={field.IsStatic}:literal={field.IsLiteral}"
            + (field.IsLiteral ? $":value={field.GetRawConstantValue()}" : ""),
        _ => member.ToString()!,
    };

    private static string ParameterKey(ParameterInfo parameter) =>
        $"{parameter.Name}:{Normalize(parameter.ParameterType.ToString())}"
        + $":in={parameter.IsIn}:out={parameter.IsOut}:optional={parameter.IsOptional}"
        + (parameter.HasDefaultValue ? $":default={parameter.DefaultValue}" : "");

    private static string TypeShape(Type type)
    {
        StructLayoutAttribute? layout = type.StructLayoutAttribute;
        return $"class={type.IsClass}:value={type.IsValueType}:enum={type.IsEnum}"
            + $":abstract={type.IsAbstract}:sealed={type.IsSealed}"
            + $":base={Normalize(type.BaseType?.ToString() ?? "")}"
            + $":interfaces={string.Join(',', type.GetInterfaces().Select(t => Normalize(t.ToString())).Order())}"
            + $":layout={layout?.Value}:pack={layout?.Pack}:size={layout?.Size}";
    }

    private static string Normalize(string value) => value
        .Replace("ArcCollision.Wrapper", "ArcCollision", StringComparison.Ordinal)
        .Replace("ArcCollision.Ref", "ArcCollision", StringComparison.Ordinal);
}
