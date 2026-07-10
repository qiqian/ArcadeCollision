using System;
using System.Collections.Generic;

namespace ArcCollision;

/// <summary>
/// A uniform-grid spatial hash for broadphase queries. Entities are inserted by
/// their AABB and can be queried by region or enumerated as candidate pairs.
///
/// Choose a cell size roughly equal to the average entity size for best results.
/// </summary>
public sealed class SpatialHash
{
    private readonly float _cellSize;
    private readonly float _invCellSize;
    private readonly Dictionary<long, List<int>> _cells = new();
    private readonly Dictionary<int, Aabb> _bounds = new();

    public SpatialHash(float cellSize)
    {
        if (cellSize <= 0f)
            throw new ArgumentOutOfRangeException(nameof(cellSize), "Cell size must be positive.");
        _cellSize = cellSize;
        _invCellSize = 1f / cellSize;
    }

    public float CellSize => _cellSize;
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

        _bounds[id] = bounds;
        ForEachCell(bounds, (cx, cy) =>
        {
            long key = Key(cx, cy);
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
        if (!_bounds.TryGetValue(id, out Aabb bounds))
            return;
        ForEachCell(bounds, (cx, cy) =>
        {
            long key = Key(cx, cy);
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
        var seen = new HashSet<int>();
        ForEachCell(region, (cx, cy) =>
        {
            if (_cells.TryGetValue(Key(cx, cy), out var list))
            {
                foreach (int id in list)
                {
                    if (seen.Add(id) && _bounds[id].Overlaps(region))
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

    private void ForEachCell(Aabb bounds, Action<int, int> action)
    {
        Vec2 min = bounds.Min;
        Vec2 max = bounds.Max;
        int minX = (int)MathF.Floor(min.X * _invCellSize);
        int minY = (int)MathF.Floor(min.Y * _invCellSize);
        int maxX = (int)MathF.Floor(max.X * _invCellSize);
        int maxY = (int)MathF.Floor(max.Y * _invCellSize);

        for (int cy = minY; cy <= maxY; cy++)
            for (int cx = minX; cx <= maxX; cx++)
                action(cx, cy);
    }

    private static long Key(int x, int y) => ((long)x << 32) | (uint)y;
}
