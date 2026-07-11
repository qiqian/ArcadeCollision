using System;
using System.Collections.Generic;

namespace ArcCollision;

/// <summary>
/// A uniform-grid spatial hash for broadphase queries. Entities are inserted by
/// their AABB and can be queried by region or enumerated as candidate pairs.
///
/// Bounds are stored in 24.8 fixed point and bucketing/overlap tests are pure
/// integer math (floats convert at the public boundary).
///
/// Choose a cell size roughly equal to the average entity size for best results.
/// </summary>
public sealed class SpatialHash
{
    private readonly record struct Cell(long X, long Y);

    private readonly struct Bounds
    {
        public readonly long MinX, MinY, MaxX, MaxY;

        public Bounds(Aabb box)
        {
            MinX = Fx.From(box.Center.X) - Fx.From(box.HalfExtents.X);
            MinY = Fx.From(box.Center.Y) - Fx.From(box.HalfExtents.Y);
            MaxX = Fx.From(box.Center.X) + Fx.From(box.HalfExtents.X);
            MaxY = Fx.From(box.Center.Y) + Fx.From(box.HalfExtents.Y);
        }

        public bool Overlaps(in Bounds b) =>
            MinX <= b.MaxX && b.MinX <= MaxX && MinY <= b.MaxY && b.MinY <= MaxY;
    }

    private readonly long _cellSizeFx;
    private readonly Dictionary<Cell, List<int>> _cells = new();
    private readonly Dictionary<int, Bounds> _bounds = new();

    public SpatialHash(float cellSize)
    {
        _cellSizeFx = Fx.From(cellSize);
        if (_cellSizeFx <= 0)
            throw new ArgumentOutOfRangeException(nameof(cellSize), "Cell size must be positive.");
    }

    public float CellSize => Fx.To(_cellSizeFx);
    public int Count => _bounds.Count;

    public void Clear()
    {
        _cells.Clear();
        _bounds.Clear();
    }

    /// <summary>Insert or move an entity identified by <paramref name="id"/>.</summary>
    public void Insert(int id, Aabb bounds)
    {
        if (_bounds.ContainsKey(id))
            Remove(id);

        var fx = new Bounds(bounds);
        _bounds[id] = fx;
        ForEachCell(fx, (cx, cy) =>
        {
            Cell key = new(cx, cy);
            if (!_cells.TryGetValue(key, out var list))
            {
                list = new List<int>();
                _cells[key] = list;
            }
            list.Add(id);
        });
    }

    public void Remove(int id)
    {
        if (!_bounds.TryGetValue(id, out Bounds fx))
            return;
        ForEachCell(fx, (cx, cy) =>
        {
            Cell key = new(cx, cy);
            if (_cells.TryGetValue(key, out var list))
            {
                list.Remove(id);
                if (list.Count == 0)
                    _cells.Remove(key);
            }
        });
        _bounds.Remove(id);
    }

    /// <summary>Collect the ids whose cells intersect <paramref name="region"/>.</summary>
    public void Query(Aabb region, List<int> results)
    {
        results.Clear();
        var fx = new Bounds(region);
        var seen = new HashSet<int>();
        ForEachCell(fx, (cx, cy) =>
        {
            if (_cells.TryGetValue(new Cell(cx, cy), out var list))
            {
                foreach (int id in list)
                {
                    if (seen.Add(id) && _bounds[id].Overlaps(fx))
                        results.Add(id);
                }
            }
        });
    }

    public List<int> Query(Aabb region)
    {
        var results = new List<int>();
        Query(region, results);
        return results;
    }

    /// <summary>
    /// Enumerate unique candidate pairs (broadphase). Only pairs whose AABBs
    /// actually overlap are returned, so this is safe to feed straight into the
    /// narrowphase.
    /// </summary>
    public IEnumerable<(int A, int B)> Pairs()
    {
        var emitted = new HashSet<long>();
        foreach (var cell in _cells.Values)
        {
            int n = cell.Count;
            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    int a = cell[i];
                    int b = cell[j];
                    if (a > b)
                        (a, b) = (b, a);

                    long pairKey = ((long)a << 32) | (uint)b;
                    if (!emitted.Add(pairKey))
                        continue;

                    if (_bounds[a].Overlaps(_bounds[b]))
                        yield return (a, b);
                }
            }
        }
    }

    private void ForEachCell(in Bounds fx, Action<long, long> action)
    {
        long minX = Fx.FloorDiv(fx.MinX, _cellSizeFx);
        long minY = Fx.FloorDiv(fx.MinY, _cellSizeFx);
        long maxX = Fx.FloorDiv(fx.MaxX, _cellSizeFx);
        long maxY = Fx.FloorDiv(fx.MaxY, _cellSizeFx);

        for (long cy = minY; cy <= maxY; cy++)
            for (long cx = minX; cx <= maxX; cx++)
                action(cx, cy);
    }
}
