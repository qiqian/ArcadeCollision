using System;
using System.Collections.Generic;

namespace ArcCollision.Ref;

/// <summary>
/// ArcWorld's internal hybrid broadphase. The historical name is retained
/// internally, but no broadphase data structure is exposed as public API.
/// </summary>
internal sealed class SpatialHash
{
    private readonly long _fatMarginFx;
    private readonly DynamicAabbTree _dynamicTree = new();
    private readonly StaticBvh _staticBvh = new();
    private readonly Dictionary<int, BpBounds> _staticBounds = new();
    private int[] _pairStackDynamic = new int[64];
    private int[] _pairStackStatic = new int[64];
    private bool _staticDirty;

    internal SpatialHash(float fatMargin)
    {
        _fatMarginFx = Fx.From(fatMargin);
        if (_fatMarginFx < 0)
            throw new ArgumentOutOfRangeException(nameof(fatMargin));
    }

    internal float FatMargin => Fx.To(_fatMarginFx);

    internal void EnsureCapacity(int colliderCapacity)
    {
        if (colliderCapacity < 0)
            throw new ArgumentOutOfRangeException(nameof(colliderCapacity));
        _dynamicTree.EnsureCapacity(colliderCapacity);
        _staticBounds.EnsureCapacity(colliderCapacity);
        _staticBvh.EnsureCapacity(colliderCapacity);
    }

    internal int AddDynamic(int slotIndex, in BpBounds bounds) =>
        _dynamicTree.CreateProxy(slotIndex, bounds.Expanded(_fatMarginFx));

    internal void UpdateDynamic(int proxyIndex, in BpBounds bounds) =>
        _dynamicTree.MoveProxy(proxyIndex, bounds, bounds.Expanded(_fatMarginFx));

    internal void RemoveDynamic(int proxyIndex) =>
        _dynamicTree.DestroyProxy(proxyIndex);

    internal void AddOrUpdateStatic(int slotIndex, in BpBounds bounds)
    {
        _staticBounds[slotIndex] = bounds;
        _staticDirty = true;
    }

    internal void RemoveStatic(int slotIndex)
    {
        if (_staticBounds.Remove(slotIndex))
            _staticDirty = true;
    }

    internal void QueryDynamic(in BpBounds bounds, List<int> results) =>
        _dynamicTree.Query(bounds, results);

    internal void QueryStatic(in BpBounds bounds, List<int> results)
    {
        EnsureStaticBuilt();
        _staticBvh.Query(bounds, results);
    }

    // Early-out existence tests; stop at the first accepted leaf.
    internal bool QueryDynamicAny<TAcceptor>(in BpBounds bounds, in TAcceptor acceptor)
        where TAcceptor : struct, IProxyAcceptor =>
        _dynamicTree.QueryAny(bounds, acceptor);

    internal bool QueryStaticAny<TAcceptor>(in BpBounds bounds, in TAcceptor acceptor)
        where TAcceptor : struct, IProxyAcceptor
    {
        EnsureStaticBuilt();
        return _staticBvh.QueryAny(bounds, acceptor);
    }

    internal void ComputePairs(List<(int A, int B)> results)
    {
        Throw.IfNull(results);
        results.Clear();
        _dynamicTree.ComputeSelfPairs(results);
        BuildStatic();

        int dynamicRoot = _dynamicTree.RootIndex;
        int staticRoot = _staticBvh.RootIndex;
        if (dynamicRoot == -1 || staticRoot == -1)
            return;

        int count = 0;
        PushPair(ref count, dynamicRoot, staticRoot);
        while (count != 0)
        {
            count--;
            int dynamicNode = _pairStackDynamic[count];
            int staticNode = _pairStackStatic[count];
            BpBounds dynamicBounds = _dynamicTree.BoundsAt(dynamicNode);
            BpBounds staticBounds = _staticBvh.BoundsAt(staticNode);
            if (!dynamicBounds.Overlaps(staticBounds))
                continue;

            bool dynamicLeaf = _dynamicTree.IsLeaf(dynamicNode);
            bool staticLeaf = _staticBvh.IsLeaf(staticNode);
            if (dynamicLeaf && staticLeaf)
            {
                results.Add((_dynamicTree.IdAt(dynamicNode), _staticBvh.IdAt(staticNode)));
                continue;
            }

            if (staticLeaf
                || (!dynamicLeaf && dynamicBounds.Perimeter >= staticBounds.Perimeter))
            {
                PushPair(ref count, _dynamicTree.Child1At(dynamicNode), staticNode);
                PushPair(ref count, _dynamicTree.Child2At(dynamicNode), staticNode);
            }
            else
            {
                PushPair(ref count, dynamicNode, _staticBvh.Child1At(staticNode));
                PushPair(ref count, dynamicNode, _staticBvh.Child2At(staticNode));
            }
        }
    }

    internal void BuildStatic()
    {
        if (!_staticDirty) return;
        _staticBvh.Build(_staticBounds);
        _staticDirty = false;
    }

    internal void Clear()
    {
        _dynamicTree.Clear();
        _staticBvh.Clear();
        _staticBounds.Clear();
        _staticDirty = false;
    }

    private void EnsureStaticBuilt()
    {
        BuildStatic();
    }

    private void PushPair(ref int count, int dynamicNode, int staticNode)
    {
        if (count == _pairStackDynamic.Length)
        {
            Array.Resize(ref _pairStackDynamic, _pairStackDynamic.Length * 2);
            Array.Resize(ref _pairStackStatic, _pairStackStatic.Length * 2);
        }
        _pairStackDynamic[count] = dynamicNode;
        _pairStackStatic[count] = staticNode;
        count++;
    }
}
