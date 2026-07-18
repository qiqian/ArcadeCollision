namespace ArcCollision.Wrapper;

internal static class FixedValidation
{
    internal static void ManifoldMode(ManifoldFields fields)
    {
        if (fields is < ManifoldFields.None or > ManifoldFields.All)
            throw new ArgumentOutOfRangeException(nameof(fields));
    }

    internal static long From(float value)
    {
        Scalar(value);
        return (long)MathF.Round(value * 256f);
    }

    // Boundary validation does not need to quantize. The native call performs
    // the one authoritative float->fixed conversion; rounding here as well was
    // pure overhead, especially for large sparse QueryBatch inputs.
    private static void Scalar(float value)
    {
        if (float.IsFinite(value)
            && value >= -CollisionLimits.MaxCoordinate
            && value <= CollisionLimits.MaxCoordinate)
            return;
        throw new ArgumentOutOfRangeException(
            "v",
            $"Fixed-point input must be finite and within +/-{CollisionLimits.MaxCoordinate}.");
    }

    internal static void Vec2(Vec2 value)
    {
        Scalar(value.X);
        Scalar(value.Y);
    }

    internal static void Circle(Circle value)
    {
        Vec2(value.Center);
        Scalar(value.Radius);
    }

    internal static void Aabb(Aabb value)
    {
        Vec2(value.Center);
        Vec2(value.HalfExtents);
    }

    internal static void Capsule(Capsule value)
    {
        Vec2(value.A);
        Vec2(value.B);
        Scalar(value.Radius);
    }

    internal static void Obb(Obb value)
    {
        Vec2(value.Center);
        Vec2(value.HalfExtents);
    }

    internal static void PolygonTransform(in Shape value)
    {
        Vec2(value.PolygonTranslation);
    }

    internal static void Shape(in Shape value)
    {
        switch (value.Kind)
        {
            case ShapeKind.Circle:
                value.TryGetCircle(out Circle circle);
                Circle(circle);
                break;
            case ShapeKind.Aabb:
                value.TryGetAabb(out Aabb box);
                Aabb(box);
                break;
            case ShapeKind.Capsule:
                value.TryGetCapsule(out Capsule capsule);
                Capsule(capsule);
                break;
            case ShapeKind.Obb:
                value.TryGetObb(out Obb obb);
                Obb(obb);
                break;
            case ShapeKind.Polygon:
                PolygonTransform(value);
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(value));
        }
    }
}
