using System;
using System.Collections.Generic;

namespace ArcCollision;

/// <summary>
/// Integer hybrid broadphase: a global immutable BVH for static geometry and a
/// balanced Dynamic AABB Tree with fat proxies for moving objects. The class
/// name is retained for source compatibility with the former spatial hash.
/// </summary>
public sealed class SpatialHash
{
    private readonly long _fatMarginFx;
    private readonly DynamicAabbTree _dynamicTree = new();
    private readonly StaticBvh _staticBvh = new();
    private readonly Dictionary<int, BpBounds> _dynamicBounds = new();
    private readonly Dictionary<int, BpBounds> _staticBounds = new();
    private readonly List<int> _candidates = new();
    private bool _staticDirty;

    /// <param name="cellSize">
    /// Compatibility parameter now used as the Dynamic Tree fat-AABB margin.
    /// Larger values reduce reinsertion frequency but produce more candidates.
    /// </param>
    public SpatialHash(float cellSize)
    {
        _fatMarginFx = Fx.From(cellSize);
        if (_fatMarginFx < 0)
            throw new ArgumentOutOfRangeException(nameof(cellSize), "Fat margin cannot be negative.");
    }

    public float CellSize => Fx.To(_fatMarginFx);
    public float FatMargin => Fx.To(_fatMarginFx);
    public int Count => _dynamicBounds.Count + _staticBounds.Count;
    public int DynamicCount => _dynamicBounds.Count;
    public int StaticCount => _staticBounds.Count;

    public void Clear()
    {
        _dynamicTree.Clear();
        _staticBvh.Clear();
        _dynamicBounds.Clear();
        _staticBounds.Clear();
        _staticDirty = false;
    }

    public void ClearDynamic()
    {
        _dynamicTree.Clear();
        _dynamicBounds.Clear();
    }

    public void ClearStatic()
    {
        _staticBvh.Clear();
        _staticBounds.Clear();
        _staticDirty = false;
    }

    public void Insert(int id, Aabb bounds) => InsertDynamic(id, new BpBounds(bounds));
    public void Insert(int id, Circle bounds) => InsertDynamic(id, new BpBounds(bounds));
    public void Insert(int id, Capsule bounds) => InsertDynamic(id, new BpBounds(bounds));
    public void Insert(int id, Obb bounds) => InsertDynamic(id, new BpBounds(bounds));

    public void Update(int id, Aabb bounds) => UpdateDynamic(id, new BpBounds(bounds));
    public void Update(int id, Circle bounds) => UpdateDynamic(id, new BpBounds(bounds));
    public void Update(int id, Capsule bounds) => UpdateDynamic(id, new BpBounds(bounds));
    public void Update(int id, Obb bounds) => UpdateDynamic(id, new BpBounds(bounds));

    public void InsertStatic(int id, Aabb bounds) => InsertStaticBounds(id, new BpBounds(bounds));
    public void InsertStatic(int id, Circle bounds) => InsertStaticBounds(id, new BpBounds(bounds));
    public void InsertStatic(int id, Capsule bounds) => InsertStaticBounds(id, new BpBounds(bounds));
    public void InsertStatic(int id, Obb bounds) => InsertStaticBounds(id, new BpBounds(bounds));

    public void UpdateStatic(int id, Aabb bounds) => InsertStaticBounds(id, new BpBounds(bounds));
    public void UpdateStatic(int id, Circle bounds) => InsertStaticBounds(id, new BpBounds(bounds));
    public void UpdateStatic(int id, Capsule bounds) => InsertStaticBounds(id, new BpBounds(bounds));
    public void UpdateStatic(int id, Obb bounds) => InsertStaticBounds(id, new BpBounds(bounds));

    /// <summary>Immediately rebuilds the immutable global static BVH.</summary>
    public void BuildStatic()
    {
        _staticBvh.Build(_staticBounds);
        _staticDirty = false;
    }

    public void Remove(int id)
    {
        if (_dynamicBounds.Remove(id))
        {
            _dynamicTree.Remove(id);
            return;
        }
        if (_staticBounds.Remove(id))
            _staticDirty = true;
    }

    public void Query(Aabb region, List<int> results) => QueryBounds(new BpBounds(region), results);
    public void Query(Circle region, List<int> results) => QueryBounds(new BpBounds(region), results);
    public void Query(Capsule region, List<int> results) => QueryBounds(new BpBounds(region), results);
    public void Query(Obb region, List<int> results) => QueryBounds(new BpBounds(region), results);

    public List<int> Query(Aabb region) => QueryNew(new BpBounds(region));
    public List<int> Query(Circle region) => QueryNew(new BpBounds(region));
    public List<int> Query(Capsule region) => QueryNew(new BpBounds(region));
    public List<int> Query(Obb region) => QueryNew(new BpBounds(region));

    /// <summary>
    /// Computes dynamic-dynamic and dynamic-static pairs. Static-static pairs
    /// are intentionally omitted because immutable geometry cannot create new
    /// contacts without a dynamic participant.
    /// </summary>
    public void ComputePairs(List<(int A, int B)> results)
    {
        ArgumentNullException.ThrowIfNull(results);
        EnsureStaticBuilt();
        results.Clear();

        foreach (KeyValuePair<int, BpBounds> item in _dynamicBounds)
        {
            int id = item.Key;
            BpBounds bounds = item.Value;

            _candidates.Clear();
            _dynamicTree.Query(bounds, _candidates);
            for (int i = 0; i < _candidates.Count; i++)
            {
                int other = _candidates[i];
                if (id < other && _dynamicBounds[other].Overlaps(bounds))
                    results.Add((id, other));
            }

            _candidates.Clear();
            _staticBvh.Query(bounds, _candidates);
            for (int i = 0; i < _candidates.Count; i++)
            {
                int other = _candidates[i];
                if (_staticBounds[other].Overlaps(bounds))
                    results.Add(id < other ? (id, other) : (other, id));
            }
        }
    }

    /// <summary>Compatibility API. Prefer <see cref="ComputePairs"/> in hot loops.</summary>
    public IEnumerable<(int A, int B)> Pairs()
    {
        var results = new List<(int A, int B)>();
        ComputePairs(results);
        return results;
    }

    private void InsertDynamic(int id, BpBounds bounds)
    {
        if (_staticBounds.Remove(id))
            _staticDirty = true;

        if (_dynamicBounds.ContainsKey(id))
        {
            UpdateDynamic(id, bounds);
            return;
        }

        _dynamicBounds.Add(id, bounds);
        _dynamicTree.Insert(id, bounds.Expanded(_fatMarginFx));
    }

    private void UpdateDynamic(int id, BpBounds bounds)
    {
        if (!_dynamicBounds.ContainsKey(id))
        {
            InsertDynamic(id, bounds);
            return;
        }

        _dynamicBounds[id] = bounds;
        _dynamicTree.Move(id, bounds, bounds.Expanded(_fatMarginFx));
    }

    private void InsertStaticBounds(int id, BpBounds bounds)
    {
        if (_dynamicBounds.Remove(id))
            _dynamicTree.Remove(id);
        _staticBounds[id] = bounds;
        _staticDirty = true;
    }

    private void QueryBounds(BpBounds bounds, List<int> results)
    {
        ArgumentNullException.ThrowIfNull(results);
        EnsureStaticBuilt();
        results.Clear();

        _candidates.Clear();
        _dynamicTree.Query(bounds, _candidates);
        for (int i = 0; i < _candidates.Count; i++)
        {
            int id = _candidates[i];
            if (_dynamicBounds[id].Overlaps(bounds))
                results.Add(id);
        }

        _candidates.Clear();
        _staticBvh.Query(bounds, _candidates);
        for (int i = 0; i < _candidates.Count; i++)
        {
            int id = _candidates[i];
            if (_staticBounds[id].Overlaps(bounds))
                results.Add(id);
        }
    }

    private List<int> QueryNew(BpBounds bounds)
    {
        var results = new List<int>();
        QueryBounds(bounds, results);
        return results;
    }

    private void EnsureStaticBuilt()
    {
        if (_staticDirty)
            BuildStatic();
    }
}
