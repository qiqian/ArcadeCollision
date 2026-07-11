using System;
using System.Collections.Generic;

namespace ArcCollision;

internal sealed class StaticBvh
{
    private const int BinCount = 12;

    private struct Leaf
    {
        public int Id;
        public BpBounds Bounds;
    }

    private struct Node
    {
        public BpBounds Bounds;
        public int Child1;
        public int Child2;
        public int Id;
        public readonly bool IsLeaf => Child1 == -1;
    }

    private struct Bin
    {
        public BpBounds Bounds;
        public int Count;
        public bool HasBounds;

        public void Add(in BpBounds bounds)
        {
            Bounds = HasBounds ? BpBounds.Union(Bounds, bounds) : bounds;
            HasBounds = true;
            Count++;
        }
    }

    private sealed class XComparer : IComparer<Leaf>
    {
        public static readonly XComparer Instance = new();
        public int Compare(Leaf a, Leaf b) => a.Bounds.CenterX.CompareTo(b.Bounds.CenterX);
    }

    private sealed class YComparer : IComparer<Leaf>
    {
        public static readonly YComparer Instance = new();
        public int Compare(Leaf a, Leaf b) => a.Bounds.CenterY.CompareTo(b.Bounds.CenterY);
    }

    private Leaf[] _leaves = Array.Empty<Leaf>();
    private Node[] _nodes = Array.Empty<Node>();
    private int[] _queryStack = new int[64];
    private int _nodeCount;
    private int _root = -1;

    public void Clear()
    {
        _root = -1;
        _nodeCount = 0;
        _leaves = Array.Empty<Leaf>();
        _nodes = Array.Empty<Node>();
    }

    public void Build(Dictionary<int, BpBounds> source)
    {
        int count = source.Count;
        if (count == 0)
        {
            Clear();
            return;
        }

        if (_leaves.Length != count)
            _leaves = new Leaf[count];
        int leafIndex = 0;
        foreach (KeyValuePair<int, BpBounds> item in source)
            _leaves[leafIndex++] = new Leaf { Id = item.Key, Bounds = item.Value };

        int requiredNodes = count * 2 - 1;
        if (_nodes.Length < requiredNodes)
            _nodes = new Node[requiredNodes];
        _nodeCount = 0;
        _root = BuildRange(0, count);
    }

    public void Query(in BpBounds bounds, List<int> results)
    {
        if (_root == -1)
            return;

        int count = 0;
        Push(ref count, _root);
        while (count != 0)
        {
            ref Node node = ref _nodes[_queryStack[--count]];
            if (!node.Bounds.Overlaps(bounds))
                continue;
            if (node.IsLeaf)
                results.Add(node.Id);
            else
            {
                Push(ref count, node.Child1);
                Push(ref count, node.Child2);
            }
        }
    }

    private int BuildRange(int start, int count)
    {
        int nodeIndex = _nodeCount++;
        if (count == 1)
        {
            Leaf leaf = _leaves[start];
            _nodes[nodeIndex] = new Node
            {
                Bounds = leaf.Bounds,
                Child1 = -1,
                Child2 = -1,
                Id = leaf.Id,
            };
            return nodeIndex;
        }

        BpBounds bounds = _leaves[start].Bounds;
        long minCenterX = bounds.CenterX, maxCenterX = bounds.CenterX;
        long minCenterY = bounds.CenterY, maxCenterY = bounds.CenterY;
        for (int i = start + 1; i < start + count; i++)
        {
            BpBounds leafBounds = _leaves[i].Bounds;
            bounds = BpBounds.Union(bounds, leafBounds);
            minCenterX = Math.Min(minCenterX, leafBounds.CenterX);
            maxCenterX = Math.Max(maxCenterX, leafBounds.CenterX);
            minCenterY = Math.Min(minCenterY, leafBounds.CenterY);
            maxCenterY = Math.Max(maxCenterY, leafBounds.CenterY);
        }

        FindSplit(start, count, minCenterX, maxCenterX, minCenterY, maxCenterY,
            out int axis, out int splitBin);
        int middle = Partition(start, count, axis, splitBin,
            axis == 0 ? minCenterX : minCenterY,
            axis == 0 ? maxCenterX : maxCenterY);

        if (middle == start || middle == start + count)
        {
            Array.Sort(_leaves, start, count, axis == 0 ? XComparer.Instance : YComparer.Instance);
            middle = start + count / 2;
        }

        int child1 = BuildRange(start, middle - start);
        int child2 = BuildRange(middle, start + count - middle);
        _nodes[nodeIndex] = new Node
        {
            Bounds = bounds,
            Child1 = child1,
            Child2 = child2,
            Id = -1,
        };
        return nodeIndex;
    }

    private void FindSplit(
        int start, int count,
        long minX, long maxX, long minY, long maxY,
        out int bestAxis, out int bestSplit)
    {
        long bestCost = long.MaxValue;
        bestAxis = maxX - minX >= maxY - minY ? 0 : 1;
        bestSplit = BinCount / 2 - 1;

        Span<Bin> bins = stackalloc Bin[BinCount];
        Span<BpBounds> leftBounds = stackalloc BpBounds[BinCount - 1];
        Span<BpBounds> rightBounds = stackalloc BpBounds[BinCount - 1];
        Span<int> leftCounts = stackalloc int[BinCount - 1];
        Span<int> rightCounts = stackalloc int[BinCount - 1];

        for (int axis = 0; axis < 2; axis++)
        {
            long min = axis == 0 ? minX : minY;
            long max = axis == 0 ? maxX : maxY;
            if (min == max)
                continue;

            bins.Clear();
            for (int i = start; i < start + count; i++)
            {
                long center = axis == 0 ? _leaves[i].Bounds.CenterX : _leaves[i].Bounds.CenterY;
                bins[ToBin(center, min, max)].Add(_leaves[i].Bounds);
            }

            BpBounds accumulated = default;
            int accumulatedCount = 0;
            bool hasBounds = false;
            for (int i = 0; i < BinCount - 1; i++)
            {
                if (bins[i].HasBounds)
                {
                    accumulated = hasBounds ? BpBounds.Union(accumulated, bins[i].Bounds) : bins[i].Bounds;
                    hasBounds = true;
                }
                accumulatedCount += bins[i].Count;
                leftBounds[i] = accumulated;
                leftCounts[i] = accumulatedCount;
            }

            accumulated = default;
            accumulatedCount = 0;
            hasBounds = false;
            for (int i = BinCount - 1; i > 0; i--)
            {
                if (bins[i].HasBounds)
                {
                    accumulated = hasBounds ? BpBounds.Union(accumulated, bins[i].Bounds) : bins[i].Bounds;
                    hasBounds = true;
                }
                accumulatedCount += bins[i].Count;
                rightBounds[i - 1] = accumulated;
                rightCounts[i - 1] = accumulatedCount;
            }

            for (int split = 0; split < BinCount - 1; split++)
            {
                if (leftCounts[split] == 0 || rightCounts[split] == 0)
                    continue;
                long cost = leftBounds[split].Perimeter * leftCounts[split]
                    + rightBounds[split].Perimeter * rightCounts[split];
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestSplit = split;
                }
            }
        }
    }

    private int Partition(int start, int count, int axis, int splitBin, long min, long max)
    {
        int left = start;
        int right = start + count - 1;
        while (left <= right)
        {
            long center = axis == 0 ? _leaves[left].Bounds.CenterX : _leaves[left].Bounds.CenterY;
            if (ToBin(center, min, max) <= splitBin)
            {
                left++;
            }
            else
            {
                (_leaves[left], _leaves[right]) = (_leaves[right], _leaves[left]);
                right--;
            }
        }
        return left;
    }

    private static int ToBin(long center, long min, long max)
    {
        long range = max - min;
        if (range <= 0)
            return 0;
        int bin = (int)(((center - min) * BinCount) / (range + 1));
        return Math.Min(bin, BinCount - 1);
    }

    private void Push(ref int count, int value)
    {
        if (count == _queryStack.Length)
            Array.Resize(ref _queryStack, _queryStack.Length * 2);
        _queryStack[count++] = value;
    }
}
