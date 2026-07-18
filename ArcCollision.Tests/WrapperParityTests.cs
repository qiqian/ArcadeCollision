using System.Reflection;
using System.Runtime.InteropServices;
using Ref = ArcCollision.Ref;
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
                referenceType.FullName!.Replace("ArcCollision.Ref", "ArcCollision.Wrapper", StringComparison.Ordinal),
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
    public void CapsulePolygonContactUsesTheLocalCapsuleSideFeature()
    {
        // Regression for test/capsule-vs-polygon.arc-case.json. The SAT normal is
        // perpendicular to the capsule spine, so both spine endpoints belong to
        // the support feature. Choosing one endpoint from a tiny Q1.30 projection
        // difference used to report a contact near (465,445), far from the actual
        // polygon corner at (438,415).
        var refCapsule = new Ref.Capsule(
            new Ref.Vec2(406, 422), new Ref.Vec2(461, 568), 34);
        var nativeCapsule = new Native.Capsule(
            new Native.Vec2(406, 422), new Native.Vec2(461, 568), 34);
        var refPolygon = new Ref.Polygon(
            new Ref.Vec2(448, 290), new Ref.Vec2(578, 275),
            new Ref.Vec2(628, 365), new Ref.Vec2(573, 445),
            new Ref.Vec2(438, 415));
        var nativePolygon = new Native.Polygon(
            new Native.Vec2(448, 290), new Native.Vec2(578, 275),
            new Native.Vec2(628, 365), new Native.Vec2(573, 445),
            new Native.Vec2(438, 415));

        Ref.Manifold expected = Ref.Collide.ShapeVsShape(
            new Ref.Shape(refCapsule), new Ref.Shape(refPolygon));
        Native.Manifold actual = Native.Collide.ShapeVsShape(
            new Native.Shape(nativeCapsule), new Native.Shape(nativePolygon));
        Ref.Manifold reverseExpected = Ref.Collide.ShapeVsShape(
            new Ref.Shape(refPolygon), new Ref.Shape(refCapsule));
        Native.Manifold reverseActual = Native.Collide.ShapeVsShape(
            new Native.Shape(nativePolygon), new Native.Shape(nativeCapsule));

        Assert.True(expected.Colliding && actual.Colliding);
        Assert.InRange(expected.Contact.X, 437f, 440f);
        Assert.InRange(expected.Contact.Y, 413f, 416f);
        AssertFloatBits(expected.Depth, actual.Depth);
        AssertVecBits(expected.Normal, actual.Normal);
        AssertVecBits(expected.Contact, actual.Contact);

        Assert.True(reverseExpected.Colliding && reverseActual.Colliding);
        Assert.InRange(reverseExpected.Contact.X, 437f, 440f);
        Assert.InRange(reverseExpected.Contact.Y, 413f, 416f);
        AssertFloatBits(reverseExpected.Depth, reverseActual.Depth);
        AssertVecBits(reverseExpected.Normal, reverseActual.Normal);
        AssertVecBits(reverseExpected.Contact, reverseActual.Contact);
    }

    [Fact]
    public void CapsuleAabbContactUsesTheLocalCapsuleSideFeature()
    {
        // Regression for test/capsule-vs-aabb.arc-case.json. The capsule side
        // grazes the box's top-right corner. Selecting one support endpoint
        // reported (379.5, 482), on the far side of the overlap, instead of the
        // actual local feature near (418, 482).
        var refCapsule = new Ref.Capsule(
            new Ref.Vec2(367, 370), new Ref.Vec2(477, 500), 34);
        var nativeCapsule = new Native.Capsule(
            new Native.Vec2(367, 370), new Native.Vec2(477, 500), 34);
        var refBox = new Ref.Aabb(
            new Ref.Vec2(338, 572), new Ref.Vec2(80, 90));
        var nativeBox = new Native.Aabb(
            new Native.Vec2(338, 572), new Native.Vec2(80, 90));

        Ref.Manifold expected = Ref.Collide.CapsuleVsAabb(refCapsule, refBox);
        Native.Manifold actual = Native.Collide.CapsuleVsAabb(nativeCapsule, nativeBox);
        Ref.Manifold genericExpected = Ref.Collide.ShapeVsShape(
            new Ref.Shape(refCapsule), new Ref.Shape(refBox));
        Native.Manifold genericActual = Native.Collide.ShapeVsShape(
            new Native.Shape(nativeCapsule), new Native.Shape(nativeBox));

        Assert.True(expected.Colliding && actual.Colliding);
        Assert.InRange(expected.Contact.X, 417f, 419f);
        Assert.InRange(expected.Contact.Y, 481f, 483f);
        AssertFloatBits(expected.Depth, actual.Depth);
        AssertVecBits(expected.Normal, actual.Normal);
        AssertVecBits(expected.Contact, actual.Contact);
        AssertFloatBits(expected.Contact.X, genericExpected.Contact.X);
        AssertFloatBits(expected.Contact.Y, genericExpected.Contact.Y);
        AssertFloatBits(actual.Contact.X, genericActual.Contact.X);
        AssertFloatBits(actual.Contact.Y, genericActual.Contact.Y);
    }

    [Fact]
    public void AabbObbContactUsesTheLocalBoxSupportFeatures()
    {
        // Regression for test/aabb-vs-obb.arc-case.json. The OBB's leftmost
        // vertex grazes the AABB's right face. Choosing the AABB's arbitrary
        // right-bottom support corner reported y=471.1 instead of the local
        // feature intersection near y=427.3.
        var refAabb = new Ref.Aabb(
            new Ref.Vec2(438, 425), new Ref.Vec2(80, 90));
        var nativeAabb = new Native.Aabb(
            new Native.Vec2(438, 425), new Native.Vec2(80, 90));
        var refObb = new Ref.Obb(
            new Ref.Vec2(627, 430), new Ref.Vec2(95, 55), -0.5f);
        var nativeObb = new Native.Obb(
            new Native.Vec2(627, 430), new Native.Vec2(95, 55), -0.5f);

        Ref.Manifold expected = Ref.Collide.ShapeVsShape(
            new Ref.Shape(refAabb), new Ref.Shape(refObb));
        Native.Manifold actual = Native.Collide.ShapeVsShape(
            new Native.Shape(nativeAabb), new Native.Shape(nativeObb));
        Ref.Manifold reverseExpected = Ref.Collide.ShapeVsShape(
            new Ref.Shape(refObb), new Ref.Shape(refAabb));
        Native.Manifold reverseActual = Native.Collide.ShapeVsShape(
            new Native.Shape(nativeObb), new Native.Shape(nativeAabb));

        Assert.True(expected.Colliding && actual.Colliding);
        Assert.InRange(expected.Contact.X, 517f, 519f);
        Assert.InRange(expected.Contact.Y, 426f, 429f);
        AssertFloatBits(expected.Depth, actual.Depth);
        AssertVecBits(expected.Normal, actual.Normal);
        AssertVecBits(expected.Contact, actual.Contact);

        Assert.True(reverseExpected.Colliding && reverseActual.Colliding);
        AssertFloatBits(reverseExpected.Depth, reverseActual.Depth);
        AssertVecBits(reverseExpected.Normal, reverseActual.Normal);
        AssertVecBits(reverseExpected.Contact, reverseActual.Contact);
        AssertFloatBits(expected.Contact.X, reverseExpected.Contact.X);
        AssertFloatBits(expected.Contact.Y, reverseExpected.Contact.Y);
    }

    [Fact]
    public void AabbPolygonContactUsesTheLocalSupportFeatures()
    {
        // Regression for test/aabb-vs-polygon.arc-case.json. The AABB's
        // top-left vertex grazes the polygon's p2-p3 support edge. Treating
        // that edge as one arbitrary endpoint reported (705,496), well below
        // the local intersection near (705,472).
        var refAabb = new Ref.Aabb(
            new Ref.Vec2(785, 562), new Ref.Vec2(80, 90));
        var nativeAabb = new Native.Aabb(
            new Native.Vec2(785, 562), new Native.Vec2(80, 90));
        var refPolygon = new Ref.Polygon(
            new Ref.Vec2(542, 365), new Ref.Vec2(682, 350),
            new Ref.Vec2(732, 450), new Ref.Vec2(647, 520),
            new Ref.Vec2(532, 475));
        var nativePolygon = new Native.Polygon(
            new Native.Vec2(542, 365), new Native.Vec2(682, 350),
            new Native.Vec2(732, 450), new Native.Vec2(647, 520),
            new Native.Vec2(532, 475));

        Ref.Manifold expected = Ref.Collide.ShapeVsShape(
            new Ref.Shape(refAabb), new Ref.Shape(refPolygon));
        Native.Manifold actual = Native.Collide.ShapeVsShape(
            new Native.Shape(nativeAabb), new Native.Shape(nativePolygon));
        Ref.Manifold reverseExpected = Ref.Collide.ShapeVsShape(
            new Ref.Shape(refPolygon), new Ref.Shape(refAabb));
        Native.Manifold reverseActual = Native.Collide.ShapeVsShape(
            new Native.Shape(nativePolygon), new Native.Shape(nativeAabb));

        Assert.True(expected.Colliding && actual.Colliding);
        Assert.InRange(expected.Contact.X, 704f, 706f);
        Assert.InRange(expected.Contact.Y, 471f, 473f);
        AssertFloatBits(expected.Depth, actual.Depth);
        AssertVecBits(expected.Normal, actual.Normal);
        AssertVecBits(expected.Contact, actual.Contact);

        Assert.True(reverseExpected.Colliding && reverseActual.Colliding);
        AssertFloatBits(reverseExpected.Depth, reverseActual.Depth);
        AssertVecBits(reverseExpected.Normal, reverseActual.Normal);
        AssertVecBits(reverseExpected.Contact, reverseActual.Contact);
        AssertFloatBits(expected.Contact.X, reverseExpected.Contact.X);
        AssertFloatBits(expected.Contact.Y, reverseExpected.Contact.Y);
    }

    [Fact]
    public void CapsuleCapsuleContactUsesTheLocalSupportFeatures()
    {
        // Regression for test/capsule-vs-capsule.arc-case.json. Capsule A's
        // lower cap grazes capsule B's long side. A tiny endpoint projection
        // difference selected B's far cap and reported (566.2,384) instead of
        // the local contact near (592.2,363.4).
        var refA = new Ref.Capsule(
            new Ref.Vec2(481, 210), new Ref.Vec2(561, 350), 34);
        var nativeA = new Native.Capsule(
            new Native.Vec2(481, 210), new Native.Vec2(561, 350), 34);
        var refB = new Ref.Capsule(
            new Ref.Vec2(637, 360), new Ref.Vec2(577, 500), 40);
        var nativeB = new Native.Capsule(
            new Native.Vec2(637, 360), new Native.Vec2(577, 500), 40);

        Ref.Manifold expected = Ref.Collide.CapsuleVsCapsule(refA, refB);
        Native.Manifold actual = Native.Collide.CapsuleVsCapsule(nativeA, nativeB);
        Ref.Manifold genericExpected = Ref.Collide.ShapeVsShape(
            new Ref.Shape(refA), new Ref.Shape(refB));
        Native.Manifold genericActual = Native.Collide.ShapeVsShape(
            new Native.Shape(nativeA), new Native.Shape(nativeB));
        Ref.Manifold reverseExpected = Ref.Collide.ShapeVsShape(
            new Ref.Shape(refB), new Ref.Shape(refA));
        Native.Manifold reverseActual = Native.Collide.ShapeVsShape(
            new Native.Shape(nativeB), new Native.Shape(nativeA));

        _ = Ref.Distance.ClosestPointsSegmentSegment(
            refA.A, refA.B, refB.A, refB.B,
            out Ref.Vec2 refClosestA, out Ref.Vec2 refClosestB);
        _ = Native.Distance.ClosestPointsSegmentSegment(
            nativeA.A, nativeA.B, nativeB.A, nativeB.B,
            out Native.Vec2 nativeClosestA, out Native.Vec2 nativeClosestB);
        Ref.Manifold refReduced = Ref.Collide.CircleVsCircle(
            new Ref.Circle(refClosestA, refA.Radius),
            new Ref.Circle(refClosestB, refB.Radius));
        Native.Manifold nativeReduced = Native.Collide.CircleVsCircle(
            new Native.Circle(nativeClosestA, nativeA.Radius),
            new Native.Circle(nativeClosestB, nativeB.Radius));

        Assert.True(expected.Colliding && actual.Colliding);
        Assert.InRange(expected.Contact.X, 591f, 593f);
        Assert.InRange(expected.Contact.Y, 362f, 365f);
        AssertFloatBits(expected.Depth, actual.Depth);
        AssertVecBits(expected.Normal, actual.Normal);
        AssertVecBits(expected.Contact, actual.Contact);
        AssertFloatBits(expected.Contact.X, genericExpected.Contact.X);
        AssertFloatBits(expected.Contact.Y, genericExpected.Contact.Y);
        AssertFloatBits(actual.Contact.X, genericActual.Contact.X);
        AssertFloatBits(actual.Contact.Y, genericActual.Contact.Y);

        // Disjoint capsule spines use the closest-points circle reduction for
        // normal/depth, with exactly the same fixed-point rounding in each backend.
        AssertFloatBits(refReduced.Depth, expected.Depth);
        AssertFloatBits(refReduced.Normal.X, expected.Normal.X);
        AssertFloatBits(refReduced.Normal.Y, expected.Normal.Y);
        AssertFloatBits(nativeReduced.Depth, actual.Depth);
        AssertFloatBits(nativeReduced.Normal.X, actual.Normal.X);
        AssertFloatBits(nativeReduced.Normal.Y, actual.Normal.Y);

        Assert.True(reverseExpected.Colliding && reverseActual.Colliding);
        AssertFloatBits(expected.Contact.X, reverseExpected.Contact.X);
        AssertFloatBits(expected.Contact.Y, reverseExpected.Contact.Y);
        AssertVecBits(reverseExpected.Contact, reverseActual.Contact);
    }

    [Fact]
    public void CrossingCapsuleSpinesKeepTheMinkowskiMtv()
    {
        var refHorizontal = new Ref.Capsule(
            new Ref.Vec2(-5, 0), new Ref.Vec2(5, 0), 1);
        var refVertical = new Ref.Capsule(
            new Ref.Vec2(0, -5), new Ref.Vec2(0, 5), 1);
        var nativeHorizontal = new Native.Capsule(
            new Native.Vec2(-5, 0), new Native.Vec2(5, 0), 1);
        var nativeVertical = new Native.Capsule(
            new Native.Vec2(0, -5), new Native.Vec2(0, 5), 1);

        Ref.Manifold expected = Ref.Collide.CapsuleVsCapsule(
            refHorizontal, refVertical);
        Native.Manifold actual = Native.Collide.CapsuleVsCapsule(
            nativeHorizontal, nativeVertical);

        Assert.True(expected.Colliding && actual.Colliding);
        // A closest-point circle reduction would report depth 2 here. Depth 7
        // proves that intersecting spines stayed on the full Minkowski SAT path.
        Assert.Equal(7f, expected.Depth, 2f / 256f);
        AssertFloatBits(expected.Depth, actual.Depth);
        AssertVecBits(expected.Normal, actual.Normal);
        AssertVecBits(expected.Contact, actual.Contact);
    }

    [Fact]
    public void ManifoldFieldsSelectTheSameWorkAndResultsInBothBackends()
    {
        foreach (var a in CreateShapes())
        foreach (var b in CreateShapes())
        {
            Ref.Manifold refAll = Ref.Collide.ShapeVsShape(
                a.Ref, b.Ref, Ref.ManifoldFields.All);
            Native.Manifold nativeAll = Native.Collide.ShapeVsShape(
                a.Native, b.Native, Native.ManifoldFields.All);
            Ref.Manifold refNormalDepth = Ref.Collide.ShapeVsShape(
                a.Ref, b.Ref, Ref.ManifoldFields.NormalDepth);
            Native.Manifold nativeNormalDepth = Native.Collide.ShapeVsShape(
                a.Native, b.Native, Native.ManifoldFields.NormalDepth);
            Ref.Manifold refNone = Ref.Collide.ShapeVsShape(
                a.Ref, b.Ref, Ref.ManifoldFields.None);
            Native.Manifold nativeNone = Native.Collide.ShapeVsShape(
                a.Native, b.Native, Native.ManifoldFields.None);

            Assert.Equal(refAll.Colliding, refNormalDepth.Colliding);
            Assert.Equal(nativeAll.Colliding, nativeNormalDepth.Colliding);
            Assert.Equal(Ref.Collide.Overlaps(a.Ref, b.Ref), refNone.Colliding);
            Assert.Equal(Native.Collide.Overlaps(a.Native, b.Native), nativeNone.Colliding);
            Assert.Equal(refNone.Colliding, nativeNone.Colliding);

            AssertFloatBits(refAll.Depth, refNormalDepth.Depth);
            AssertFloatBits(nativeAll.Depth, nativeNormalDepth.Depth);
            AssertFloatBits(refNormalDepth.Depth, nativeNormalDepth.Depth);
            AssertFloatBits(refAll.Normal.X, refNormalDepth.Normal.X);
            AssertFloatBits(refAll.Normal.Y, refNormalDepth.Normal.Y);
            AssertVecBits(refNormalDepth.Normal, nativeNormalDepth.Normal);

            Assert.Equal(Ref.Vec2.Zero, refNormalDepth.Contact);
            Assert.Equal(Native.Vec2.Zero, nativeNormalDepth.Contact);
            Assert.Equal(Ref.Vec2.Zero, refNone.Normal);
            Assert.Equal(Native.Vec2.Zero, nativeNone.Normal);
            Assert.Equal(0f, refNone.Depth);
            Assert.Equal(0f, nativeNone.Depth);
            Assert.Equal(Ref.Vec2.Zero, refNone.Contact);
            Assert.Equal(Native.Vec2.Zero, nativeNone.Contact);
        }

        (Ref.Shape refShape, Native.Shape nativeShape) = CreateShapes()[0];
        Assert.Throws<ArgumentOutOfRangeException>(() => Ref.Collide.ShapeVsShape(
            refShape, refShape, (Ref.ManifoldFields)3));
        Assert.Throws<ArgumentOutOfRangeException>(() => Native.Collide.ShapeVsShape(
            nativeShape, nativeShape, (Native.ManifoldFields)3));
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

    [Fact]
    public void WorldTransformsMaterializeBitExactlyAcrossBackends()
    {
        (Ref.Shape Ref, Native.Shape Native)[] shapes = CreateShapes();
        for (int i = 0; i < shapes.Length; i++)
        {
            using var reference = new Ref.ArcWorld();
            using var native = new Native.ArcWorld();
            Ref.ArcHandle refHandle = reference.Add(i, shapes[i].Ref);
            Native.ArcHandle nativeHandle = native.Add(i, shapes[i].Native);

            var refAbsolute = new Ref.Transform(
                new Ref.Vec2(1_234_567.1f, -765_432.06f),
                new Ref.Angle32(0x2A17C39Du), 1.2345678f);
            var nativeAbsolute = new Native.Transform(
                new Native.Vec2(1_234_567.1f, -765_432.06f),
                new Native.Angle32(0x2A17C39Du), 1.2345678f);
            reference.UpdateTransform(refHandle, refAbsolute);
            native.UpdateTransform(nativeHandle, nativeAbsolute);
            AssertShapeBits(reference.GetShape(refHandle), native.GetShape(nativeHandle));

            var refDelta = new Ref.Transform(
                new Ref.Vec2(-17.003f, 29.997f),
                new Ref.Angle32(0x139A24E7u), .812345f);
            var nativeDelta = new Native.Transform(
                new Native.Vec2(-17.003f, 29.997f),
                new Native.Angle32(0x139A24E7u), .812345f);
            reference.UpdateTransformDelta(refHandle, refDelta);
            native.UpdateTransformDelta(nativeHandle, nativeDelta);
            AssertShapeBits(reference.GetShape(refHandle), native.GetShape(nativeHandle));
        }
    }

    [Fact]
    public void WorldIdentityRotationAndScaleMaterializeBitExactlyAcrossBackends()
    {
        var shapes = new List<(Ref.Shape Ref, Native.Shape Native)>(CreateShapes())
        {
            // Covers the native OBB final-angle-zero bounds fast path; the OBB
            // in CreateShapes intentionally has a non-zero authored angle.
            (new Ref.Obb(Ref.Vec2.Zero, new Ref.Vec2(1.25f, .75f), 0f),
             new Native.Obb(Native.Vec2.Zero, new Native.Vec2(1.25f, .75f), 0f)),
        };
        var refPosition = new Ref.Vec2(123.125f, -45.375f);
        var nativePosition = new Native.Vec2(123.125f, -45.375f);

        for (int i = 0; i < shapes.Count; i++)
        {
            using var reference = new Ref.ArcWorld();
            using var native = new Native.ArcWorld();
            Ref.ArcHandle refHandle = reference.Add(i, shapes[i].Ref);
            Native.ArcHandle nativeHandle = native.Add(i, shapes[i].Native);

            reference.UpdateTransform(refHandle, new Ref.Transform(
                refPosition, new Ref.Angle32(0), 1f));
            native.UpdateTransform(nativeHandle, new Native.Transform(
                nativePosition, new Native.Angle32(0), 1f));
            AssertShapeBits(reference.GetShape(refHandle), native.GetShape(nativeHandle));

            // Query observes the materialized bounds stored in each world, not
            // merely the public shape returned by GetShape.
            for (int y = -8; y <= 8; y++)
            {
                for (int x = -8; x <= 8; x++)
                {
                    var refHits = new List<Ref.ArcHandle>();
                    var nativeHits = new List<Native.ArcHandle>();
                    var refProbe = new Ref.Vec2(
                        refPosition.X + x * .25f, refPosition.Y + y * .25f);
                    var nativeProbe = new Native.Vec2(
                        nativePosition.X + x * .25f, nativePosition.Y + y * .25f);
                    reference.Query(new Ref.Aabb(refProbe, new Ref.Vec2(Grid, Grid)), refHits);
                    native.Query(new Native.Aabb(
                        nativeProbe, new Native.Vec2(Grid, Grid)), nativeHits);
                    Assert.Equal(refHits.Contains(refHandle), nativeHits.Contains(nativeHandle));
                }
            }
        }
    }

    [Fact]
    public void OddGridBoundsContactsAndSweepSignedZeroAreBitExact()
    {
        const float g = 1f / 256f;
        var refPolygon = new Ref.Polygon(
            new Ref.Vec2(0, 0), new Ref.Vec2(5 * g, 0),
            new Ref.Vec2(5 * g, 3 * g), new Ref.Vec2(0, 3 * g));
        var nativePolygon = new Native.Polygon(
            new Native.Vec2(0, 0), new Native.Vec2(5 * g, 0),
            new Native.Vec2(5 * g, 3 * g), new Native.Vec2(0, 3 * g));
        var angleRef = new Ref.Angle32(0x10000000u);
        var angleNative = new Native.Angle32(0x10000000u);

        var refShape = new Ref.Shape(
            refPolygon, new Ref.Vec2(-11 * g, -7 * g), angleRef);
        var nativeShape = new Native.Shape(
            nativePolygon, new Native.Vec2(-11 * g, -7 * g), angleNative);
        AssertFloatBits(refShape.Bounds.Center.X, nativeShape.Bounds.Center.X);
        AssertFloatBits(refShape.Bounds.Center.Y, nativeShape.Bounds.Center.Y);
        AssertFloatBits(refShape.Bounds.HalfExtents.X, nativeShape.Bounds.HalfExtents.X);
        AssertFloatBits(refShape.Bounds.HalfExtents.Y, nativeShape.Bounds.HalfExtents.Y);

        for (int raw = -12; raw <= 4; raw++)
        {
            var targetRef = new Ref.Aabb(
                new Ref.Vec2(raw * g, -4 * g), new Ref.Vec2(4 * g, 4 * g));
            var targetNative = new Native.Aabb(
                new Native.Vec2(raw * g, -4 * g), new Native.Vec2(4 * g, 4 * g));
            Ref.Manifold expected = Ref.Collide.ShapeVsShape(refShape, targetRef);
            Native.Manifold actual = Native.Collide.ShapeVsShape(nativeShape, targetNative);
            Assert.Equal(expected.Colliding, actual.Colliding);
            if (!expected.Colliding) continue;
            AssertFloatBits(expected.Contact.X, actual.Contact.X);
            AssertFloatBits(expected.Contact.Y, actual.Contact.Y);
        }

        Ref.SweepHit refSweep = Ref.Sweep.MovingShapeVsShape(
            new Ref.Aabb(new Ref.Vec2(-5, 0), new Ref.Vec2(1, 1)),
            new Ref.Vec2(10, 0), new Ref.Circle(Ref.Vec2.Zero, 1));
        Native.SweepHit nativeSweep = Native.Sweep.MovingShapeVsShape(
            new Native.Aabb(new Native.Vec2(-5, 0), new Native.Vec2(1, 1)),
            new Native.Vec2(10, 0), new Native.Circle(Native.Vec2.Zero, 1));
        AssertFloatBits(refSweep.Normal.X, nativeSweep.Normal.X);
        AssertFloatBits(refSweep.Normal.Y, nativeSweep.Normal.Y);
        AssertFloatBits(refSweep.Point.X, nativeSweep.Point.X);
        AssertFloatBits(refSweep.Point.Y, nativeSweep.Point.Y);
    }

    [Fact]
    public void ManagedPrimitiveHelpersAreBitExact()
    {
        Ref.Vec2 refNormalized = new Ref.Vec2(1f / 7f, 2f / 11f).Normalized();
        Native.Vec2 nativeNormalized =
            new Native.Vec2(1f / 7f, 2f / 11f).Normalized();
        AssertFloatBits(refNormalized.X, nativeNormalized.X);
        AssertFloatBits(refNormalized.Y, nativeNormalized.Y);

        AssertFloatBits(
            Ref.Manifold.None.SeparationForA.X,
            Native.Manifold.None.SeparationForA.X);
        AssertFloatBits(
            Ref.Manifold.None.SeparationForA.Y,
            Native.Manifold.None.SeparationForA.Y);

        var refNonColliding = new Ref.Manifold(
            false, new Ref.Vec2(2, -3), 4, Ref.Vec2.Zero);
        var nativeNonColliding = new Native.Manifold(
            false, new Native.Vec2(2, -3), 4, Native.Vec2.Zero);
        AssertFloatBits(
            refNonColliding.SeparationForA.X,
            nativeNonColliding.SeparationForA.X);
        AssertFloatBits(
            refNonColliding.SeparationForA.Y,
            nativeNonColliding.SeparationForA.Y);
        AssertFloatBits(
            refNonColliding.SeparationForB.X,
            nativeNonColliding.SeparationForB.X);
        AssertFloatBits(
            refNonColliding.SeparationForB.Y,
            nativeNonColliding.SeparationForB.Y);
    }

    [Fact]
    public void MovedPolygonRetainsCachedBoundsBits()
    {
        const float grid = 1f / 256f;
        var reference = new Ref.Polygon(
            Ref.Vec2.Zero, new Ref.Vec2(grid, 0), new Ref.Vec2(0, grid));
        var native = new Native.Polygon(
            Native.Vec2.Zero, new Native.Vec2(grid, 0), new Native.Vec2(0, grid));

        Ref.Aabb expected = reference.Moved(new Ref.Vec2(1_000_000, 0)).Bounds;
        Native.Aabb actual = native.Moved(new Native.Vec2(1_000_000, 0)).Bounds;
        AssertFloatBits(expected.Center.X, actual.Center.X);
        AssertFloatBits(expected.Center.Y, actual.Center.Y);
        AssertFloatBits(expected.HalfExtents.X, actual.HalfExtents.X);
        AssertFloatBits(expected.HalfExtents.Y, actual.HalfExtents.Y);
    }

    [Fact]
    public void StandaloneBroadphaseQueriesAppendToExistingLists()
    {
        var bounds = new Ref.BpBounds(-10, -10, 10, 10);
        var nativeBounds = new Native.BpBounds(-10, -10, 10, 10);
        using (var reference = new Ref.DynamicAabbTree())
        using (var native = new Native.DynamicAabbTree())
        {
            reference.CreateProxy(7, bounds);
            reference.CreateProxy(9, bounds);
            native.CreateProxy(7, nativeBounds);
            native.CreateProxy(9, nativeBounds);

            var expectedQuery = new List<int> { -1 };
            var actualQuery = new List<int> { -1 };
            reference.Query(bounds, expectedQuery);
            native.Query(nativeBounds, actualQuery);
            Assert.Equal(expectedQuery, actualQuery);

            var expectedPairs = new List<(int A, int B)> { (-3, -2) };
            var actualPairs = new List<(int A, int B)> { (-3, -2) };
            reference.ComputeSelfPairs(expectedPairs);
            native.ComputeSelfPairs(actualPairs);
            Assert.Equal(expectedPairs, actualPairs);
        }

        using (var reference = new Ref.StaticBvh())
        using (var native = new Native.StaticBvh())
        {
            reference.Build(new Dictionary<int, Ref.BpBounds> { [5] = bounds });
            native.Build(new Dictionary<int, Native.BpBounds> { [5] = nativeBounds });
            var expected = new List<int> { -1 };
            var actual = new List<int> { -1 };
            reference.Query(bounds, expected);
            native.Query(nativeBounds, actual);
            Assert.Equal(expected, actual);
        }
    }

    private static void AssertFloatBits(float expected, float actual) =>
        Assert.Equal(BitConverter.SingleToUInt32Bits(expected),
            BitConverter.SingleToUInt32Bits(actual));

    private static void AssertShapeBits(Ref.Shape expected, Native.Shape actual)
    {
        Assert.Equal((int)expected.Kind, (int)actual.Kind);
        switch (expected.Kind)
        {
            case Ref.ShapeKind.Circle:
                Assert.True(expected.TryGetCircle(out Ref.Circle refCircle));
                Assert.True(actual.TryGetCircle(out Native.Circle nativeCircle));
                AssertVecBits(refCircle.Center, nativeCircle.Center);
                AssertFloatBits(refCircle.Radius, nativeCircle.Radius);
                break;
            case Ref.ShapeKind.Aabb:
                Assert.True(expected.TryGetAabb(out Ref.Aabb refAabb));
                Assert.True(actual.TryGetAabb(out Native.Aabb nativeAabb));
                AssertVecBits(refAabb.Center, nativeAabb.Center);
                AssertVecBits(refAabb.HalfExtents, nativeAabb.HalfExtents);
                break;
            case Ref.ShapeKind.Capsule:
                Assert.True(expected.TryGetCapsule(out Ref.Capsule refCapsule));
                Assert.True(actual.TryGetCapsule(out Native.Capsule nativeCapsule));
                AssertVecBits(refCapsule.A, nativeCapsule.A);
                AssertVecBits(refCapsule.B, nativeCapsule.B);
                AssertFloatBits(refCapsule.Radius, nativeCapsule.Radius);
                break;
            case Ref.ShapeKind.Obb:
                Assert.True(expected.TryGetObb(out Ref.Obb refObb));
                Assert.True(actual.TryGetObb(out Native.Obb nativeObb));
                AssertVecBits(refObb.Center, nativeObb.Center);
                AssertVecBits(refObb.HalfExtents, nativeObb.HalfExtents);
                Assert.Equal(refObb.Angle.Raw, nativeObb.Angle.Raw);
                break;
            case Ref.ShapeKind.Polygon:
                Assert.True(expected.TryGetPolygon(
                    out Ref.Polygon? refPolygon, out Ref.Vec2 refTranslation,
                    out Ref.Angle32 refRotation));
                Assert.True(actual.TryGetPolygon(
                    out Native.Polygon? nativePolygon, out Native.Vec2 nativeTranslation,
                    out Native.Angle32 nativeRotation));
                AssertVecBits(refTranslation, nativeTranslation);
                Assert.Equal(refRotation.Raw, nativeRotation.Raw);
                Assert.Equal(refPolygon!.Count, nativePolygon!.Count);
                for (int i = 0; i < refPolygon.Count; i++)
                    AssertVecBits(refPolygon[i], nativePolygon[i]);
                break;
        }
    }

    private static void AssertVecBits(Ref.Vec2 expected, Native.Vec2 actual)
    {
        AssertFloatBits(expected.X, actual.X);
        AssertFloatBits(expected.Y, actual.Y);
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
        assembly.GetExportedTypes().Where(t => t.Namespace is "ArcCollision.Ref" or "ArcCollision.Wrapper");

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
