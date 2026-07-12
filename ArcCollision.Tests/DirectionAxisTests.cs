using System;
using Xunit;

namespace ArcCollision.Tests;

public class DirectionAxisTests
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
    public void LargeObb_DepthErrorNoLongerScalesAtQ8AxisRate()
    {
        const float angle = 0.713f;
        const float halfX = 100_000f;
        const float radius = 20f;
        const float expectedDepth = 3f;
        float c = (float)Math.Cos(angle);
        float s = (float)Math.Sin(angle);
        var box = new Obb(Vec2.Zero, new Vec2(halfX, 30_000f), angle);
        var circle = new Circle(
            new Vec2(c * (halfX + radius - expectedDepth),
                s * (halfX + radius - expectedDepth)), radius);

        Manifold hit = Collide.ShapeVsShape(circle, box);

        Assert.True(hit.Colliding);
        Assert.InRange(Math.Abs(hit.Depth - expectedDepth), 0, 6f / 256f);
        Assert.InRange(Math.Abs(hit.Normal.X + c), 0, 2e-6f);
        Assert.InRange(Math.Abs(hit.Normal.Y + s), 0, 2e-6f);
    }
}
