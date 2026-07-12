using Xunit;

namespace ArcCollision.Tests;

public class DirectionAxisTests
{
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
