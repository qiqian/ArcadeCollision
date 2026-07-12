global using ArcCollision;
using Xunit;

namespace ArcCollision.Tests;

public class FixedMathTest
{
    [Theory]
    [InlineData(1L, 1L)]
    [InlineData(1L, 2L)]
    [InlineData(2L, 1L)]
    [InlineData(-1L, 1L)]
    [InlineData(1L, -1L)]
    [InlineData(17L, 31L)]
    [InlineData(-17L, -31L)]
    [InlineData(1_000_000L, 999_999L)]
    public void FxAxis_AdaptiveNormalizationPreservesShortDirections(long x, long y)
    {
        FxAxis axis = FxAxis.FromComponents(x, y, FxAxis.UnitX);
        double length = Math.Sqrt((double)x * x + (double)y * y);
        double actualX = axis.X / (double)FxAxis.One;
        double actualY = axis.Y / (double)FxAxis.One;

        Assert.InRange(Math.Abs(actualX - x / length), 0, 4.0 / FxAxis.One);
        Assert.InRange(Math.Abs(actualY - y / length), 0, 4.0 / FxAxis.One);
        Assert.InRange(Math.Abs(Math.Sqrt(actualX * actualX + actualY * actualY) - 1.0),
            0, 6.0 / FxAxis.One);
    }

    [Fact]
    public void Angle32_CardinalAxesAreExact()
    {
        AssertAxis(new Angle32(0x00000000u), FxAxis.One, 0);
        AssertAxis(new Angle32(0x40000000u), 0, FxAxis.One);
        AssertAxis(new Angle32(0x80000000u), -FxAxis.One, 0);
        AssertAxis(new Angle32(0xC0000000u), 0, -FxAxis.One);
    }
    private static void AssertAxis(Angle32 angle, long x, long y)
    {
        FxAxis axis = FxAxis.FromAngle(angle);
        Assert.Equal(x, axis.X);
        Assert.Equal(y, axis.Y);
    }

    // box2d/test/test_collision.c: LargeWorldAABBTest. Box2D obtains the 0.6
    // extent from a 0.5 box plus 0.1 polygon radius. ArcCollision stores the
    // already-expanded shape bounds, then applies the fat margin explicitly.
    [Fact]
    public void LargeWorldAabb_Box2DFixture_PreservesTightAndFatExtents()
    {
        AssertBoundsAtBase(Vec2.Zero);
        AssertBoundsAtBase(new(1_898_000, -1_898_000));
    }

    private static void AssertBoundsAtBase(Vec2 basePosition)
    {
        var roundedBoxBounds = new Aabb(basePosition, new Vec2(0.6f, 0.6f));
        var tight = new BpBounds(roundedBoxBounds);
        long centerX = Fx.From(basePosition.X);
        long centerY = Fx.From(basePosition.Y);
        long extent = Fx.From(0.6f);

        Assert.True(tight.MinX <= centerX - extent);
        Assert.True(tight.MinY <= centerY - extent);
        Assert.True(tight.MaxX >= centerX + extent);
        Assert.True(tight.MaxY >= centerY + extent);

        long extra = Fx.From(0.05f);
        BpBounds fat = tight.Expanded(extra);
        Assert.True(fat.MinX <= centerX - extent - extra);
        Assert.True(fat.MinY <= centerY - extent - extra);
        Assert.True(fat.MaxX >= centerX + extent + extra);
        Assert.True(fat.MaxY >= centerY + extent + extra);
    }

    [Fact]
    public void Angle32_CordicIsBitExactAcrossMirrorAndQuarterTurns()
    {
        uint raw = 0xA341316Cu;
        for (int i = 0; i < 10_000; i++)
        {
            raw = unchecked(raw * 1664525u + 1013904223u);
            FxAxis axis = FxAxis.FromAngle(new Angle32(raw));
            FxAxis mirrored = FxAxis.FromAngle(new Angle32(unchecked(0u - raw)));
            FxAxis quarter = FxAxis.FromAngle(new Angle32(unchecked(raw + 0x40000000u)));

            Assert.Equal(axis.X, mirrored.X);
            Assert.Equal(-axis.Y, mirrored.Y);
            Assert.Equal(-axis.Y, quarter.X);
            Assert.Equal(axis.X, quarter.Y);

            double x = axis.X / (double)FxAxis.One;
            double y = axis.Y / (double)FxAxis.One;
            Assert.InRange(Math.Abs(x * x + y * y - 1.0), 0, 8e-9);
        }
    }
}