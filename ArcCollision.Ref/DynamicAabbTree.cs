using System;
using System.Collections.Generic;
using System.Threading;

namespace ArcCollision.Ref;

/// <summary>
/// Incremental broadphase: a self-balancing AABB tree of fat proxies supporting
/// add/move/remove and box queries. This is the reference managed implementation;
/// <see cref="ArcCollision.Wrapper"/> exposes a drop-in native equivalent with the
/// same public surface. <see cref="IDisposable"/> is implemented for parity with
/// the native wrapper (managed instances are simply GC-collected).
/// </summary>
public sealed class DynamicAabbTree : IDisposable
{
    private struct Node
    {
        public BpBounds Bounds;
        public int Parent;
        public int Child1;
        public int Child2;
        public int Height;
        public int Id;

        public readonly bool IsLeaf => Child1 == -1;
    }

    private Node[] _nodes = new Node[16];
    private int[] _queryStack = new int[64];
    private int[] _pairStackA = new int[64];
    private int[] _pairStackB = new int[64];
    private int _root = -1;
    private int _freeList;
    private int _nodeCount;
    private int _leafCount;
    private int _disposed;

    public DynamicAabbTree() => InitializeFreeList(0);

    public int Count { get { ThrowIfDisposed(); return _leafCount; } }

    public void EnsureCapacity(int proxyCapacity)
    {
        ThrowIfDisposed();
        if (proxyCapacity < 0)
            throw new ArgumentOutOfRangeException(nameof(proxyCapacity));
        int requiredNodes = proxyCapacity == 0 ? 0 : proxyCapacity * 2 - 1;
        while (_nodes.Length < requiredNodes)
            GrowNodes();
    }

    public void Clear()
    {
        ThrowIfDisposed();
        _root = -1;
        _nodeCount = 0;
        _leafCount = 0;
        InitializeFreeList(0);
    }

    /// <summary>
    /// Releases the tree. The managed implementation has nothing unmanaged to free
    /// (it just resets); the method exists so the public surface matches the native
    /// wrapper, which frees its native handle here.
    /// </summary>
    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) != 0) return;
        _root = -1;
        _nodeCount = 0;
        _leafCount = 0;
    }

    /// <summary>Creates a leaf and returns its direct proxy index.</summary>
    public int CreateProxy(int id, in BpBounds fatBounds)
    {
        ThrowIfDisposed();
        int leaf = AllocateNode();
        _nodes[leaf].Bounds = fatBounds;
        _nodes[leaf].Id = id;
        _nodes[leaf].Height = 0;
        _nodes[leaf].Child1 = -1;
        _nodes[leaf].Child2 = -1;
        InsertLeaf(leaf);
        _leafCount++;
        return leaf;
    }

    public bool MoveProxy(int proxy, in BpBounds bounds, in BpBounds fatBounds)
    {
        ThrowIfDisposed();
        if (_nodes[proxy].Bounds.Contains(bounds))
            return false;

        RemoveLeaf(proxy);
        _nodes[proxy].Bounds = fatBounds;
        InsertLeaf(proxy);
        return true;
    }

    public void DestroyProxy(int proxy)
    {
        ThrowIfDisposed();
        RemoveLeaf(proxy);
        FreeNode(proxy);
        _leafCount--;
    }

    public void Query(in BpBounds bounds, List<int> results)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(results);
        if (_root == -1)
            return;

        int count = 0;
        Push(ref count, _root);
        while (count != 0)
        {
            int index = _queryStack[--count];
            ref Node node = ref _nodes[index];
            if (!node.Bounds.Overlaps(bounds))
                continue;

            if (node.IsLeaf)
            {
                results.Add(node.Id);
            }
            else
            {
                Push(ref count, node.Child1);
                Push(ref count, node.Child2);
            }
        }
    }

    /// <summary>
    /// Existence-only box query: the same descent as <see cref="Query"/>, but it
    /// hands each overlapping leaf to <paramref name="acceptor"/> and returns as
    /// soon as one is taken, so a caller that only needs "is anything here?"
    /// skips both the result list and the rest of the traversal. The acceptor is
    /// a struct type parameter so the JIT specializes the call and no delegate or
    /// closure is allocated. A boolean is order-independent, so this needs no
    /// sort and matches the native backend regardless of descent order.
    /// </summary>
    internal bool QueryAny<TAcceptor>(in BpBounds bounds, in TAcceptor acceptor)
        where TAcceptor : struct, IProxyAcceptor
    {
        ThrowIfDisposed();
        if (_root == -1)
            return false;

        int count = 0;
        Push(ref count, _root);
        while (count != 0)
        {
            int index = _queryStack[--count];
            ref Node node = ref _nodes[index];
            if (!node.Bounds.Overlaps(bounds))
                continue;

            if (node.IsLeaf)
            {
                if (acceptor.Accept(node.Id))
                    return true;
            }
            else
            {
                Push(ref count, node.Child1);
                Push(ref count, node.Child2);
            }
        }
        return false;
    }

    /// <summary>
    /// Enumerates every pair of proxies whose fat bounds overlap, each pair
    /// ordered so the smaller id comes first. This is the standalone-broadphase
    /// capability the internal node accessors exist to serve; the raw per-node
    /// accessors themselves stay internal (walking them across the native
    /// boundary would cost one call per node).
    /// </summary>
    public void ComputeSelfPairs(List<(int A, int B)> results)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(results);
        if (_root == -1 || _leafCount < 2)
            return;

        int count = 0;
        PushPair(ref count, _root, _root);
        while (count != 0)
        {
            count--;
            int a = _pairStackA[count];
            int b = _pairStackB[count];
            ref Node nodeA = ref _nodes[a];
            ref Node nodeB = ref _nodes[b];

            if (a == b)
            {
                if (nodeA.IsLeaf) continue;
                PushPair(ref count, nodeA.Child1, nodeA.Child1);
                PushPair(ref count, nodeA.Child1, nodeA.Child2);
                PushPair(ref count, nodeA.Child2, nodeA.Child2);
                continue;
            }

            if (!nodeA.Bounds.Overlaps(nodeB.Bounds))
                continue;
            if (nodeA.IsLeaf && nodeB.IsLeaf)
            {
                results.Add(nodeA.Id < nodeB.Id
                    ? (nodeA.Id, nodeB.Id)
                    : (nodeB.Id, nodeA.Id));
                continue;
            }

            if (nodeB.IsLeaf
                || (!nodeA.IsLeaf && nodeA.Bounds.Perimeter >= nodeB.Bounds.Perimeter))
            {
                PushPair(ref count, nodeA.Child1, b);
                PushPair(ref count, nodeA.Child2, b);
            }
            else
            {
                PushPair(ref count, a, nodeB.Child1);
                PushPair(ref count, a, nodeB.Child2);
            }
        }
    }

    internal int RootIndex => _root;
    internal bool IsLeaf(int index) => _nodes[index].IsLeaf;
    internal BpBounds BoundsAt(int index) => _nodes[index].Bounds;
    internal int Child1At(int index) => _nodes[index].Child1;
    internal int Child2At(int index) => _nodes[index].Child2;
    internal int IdAt(int index) => _nodes[index].Id;

    private void InsertLeaf(int leaf)
    {
        if (_root == -1)
        {
            _root = leaf;
            _nodes[leaf].Parent = -1;
            return;
        }

        BpBounds leafBounds = _nodes[leaf].Bounds;
        int sibling = _root;
        while (!_nodes[sibling].IsLeaf)
        {
            int child1 = _nodes[sibling].Child1;
            int child2 = _nodes[sibling].Child2;
            long area = _nodes[sibling].Bounds.Perimeter;
            BpBounds combined = BpBounds.Union(_nodes[sibling].Bounds, leafBounds);
            long combinedArea = combined.Perimeter;
            long directCost = 2 * combinedArea;
            long inheritanceCost = 2 * (combinedArea - area);
            long cost1 = DescendCost(child1, leafBounds, inheritanceCost);
            long cost2 = DescendCost(child2, leafBounds, inheritanceCost);
            if (directCost < cost1 && directCost < cost2)
                break;
            sibling = cost1 < cost2 ? child1 : child2;
        }

        int oldParent = _nodes[sibling].Parent;
        int newParent = AllocateNode();
        _nodes[newParent].Parent = oldParent;
        _nodes[newParent].Bounds = BpBounds.Union(leafBounds, _nodes[sibling].Bounds);
        _nodes[newParent].Height = _nodes[sibling].Height + 1;
        _nodes[newParent].Child1 = sibling;
        _nodes[newParent].Child2 = leaf;
        _nodes[newParent].Id = -1;
        _nodes[sibling].Parent = newParent;
        _nodes[leaf].Parent = newParent;

        if (oldParent == -1)
            _root = newParent;
        else if (_nodes[oldParent].Child1 == sibling)
            _nodes[oldParent].Child1 = newParent;
        else
            _nodes[oldParent].Child2 = newParent;

        FixUpward(newParent);
    }

    private long DescendCost(int child, in BpBounds leafBounds, long inheritanceCost)
    {
        BpBounds combined = BpBounds.Union(_nodes[child].Bounds, leafBounds);
        return _nodes[child].IsLeaf
            ? combined.Perimeter + inheritanceCost
            : combined.Perimeter - _nodes[child].Bounds.Perimeter + inheritanceCost;
    }

    private void RemoveLeaf(int leaf)
    {
        if (leaf == _root)
        {
            _root = -1;
            return;
        }

        int parent = _nodes[leaf].Parent;
        int grandParent = _nodes[parent].Parent;
        int sibling = _nodes[parent].Child1 == leaf
            ? _nodes[parent].Child2
            : _nodes[parent].Child1;

        if (grandParent == -1)
        {
            _root = sibling;
            _nodes[sibling].Parent = -1;
        }
        else
        {
            if (_nodes[grandParent].Child1 == parent)
                _nodes[grandParent].Child1 = sibling;
            else
                _nodes[grandParent].Child2 = sibling;
            _nodes[sibling].Parent = grandParent;
            FixUpward(grandParent);
        }

        FreeNode(parent);
        _nodes[leaf].Parent = -1;
    }

    private void FixUpward(int index)
    {
        while (index != -1)
        {
            index = Balance(index);
            int child1 = _nodes[index].Child1;
            int child2 = _nodes[index].Child2;
            _nodes[index].Height = 1 + Math.Max(_nodes[child1].Height, _nodes[child2].Height);
            _nodes[index].Bounds = BpBounds.Union(_nodes[child1].Bounds, _nodes[child2].Bounds);
            index = _nodes[index].Parent;
        }
    }

    private int Balance(int a)
    {
        if (_nodes[a].IsLeaf || _nodes[a].Height < 2)
            return a;

        int b = _nodes[a].Child1;
        int c = _nodes[a].Child2;
        int balance = _nodes[c].Height - _nodes[b].Height;

        if (balance > 1)
        {
            int f = _nodes[c].Child1;
            int g = _nodes[c].Child2;
            _nodes[c].Child1 = a;
            ReplaceParent(a, c);
            _nodes[a].Parent = c;

            if (_nodes[f].Height > _nodes[g].Height)
            {
                _nodes[c].Child2 = f;
                _nodes[a].Child2 = g;
                _nodes[g].Parent = a;
                _nodes[f].Parent = c;
            }
            else
            {
                _nodes[c].Child2 = g;
                _nodes[a].Child2 = f;
                _nodes[f].Parent = a;
                _nodes[g].Parent = c;
            }
            Refit(a);
            Refit(c);
            return c;
        }

        if (balance < -1)
        {
            int d = _nodes[b].Child1;
            int e = _nodes[b].Child2;
            _nodes[b].Child1 = a;
            ReplaceParent(a, b);
            _nodes[a].Parent = b;

            if (_nodes[d].Height > _nodes[e].Height)
            {
                _nodes[b].Child2 = d;
                _nodes[a].Child1 = e;
                _nodes[e].Parent = a;
                _nodes[d].Parent = b;
            }
            else
            {
                _nodes[b].Child2 = e;
                _nodes[a].Child1 = d;
                _nodes[d].Parent = a;
                _nodes[e].Parent = b;
            }
            Refit(a);
            Refit(b);
            return b;
        }

        return a;
    }

    private void ReplaceParent(int oldChild, int newChild)
    {
        int parent = _nodes[oldChild].Parent;
        _nodes[newChild].Parent = parent;
        if (parent == -1)
            _root = newChild;
        else if (_nodes[parent].Child1 == oldChild)
            _nodes[parent].Child1 = newChild;
        else
            _nodes[parent].Child2 = newChild;
    }

    private void Refit(int index)
    {
        int child1 = _nodes[index].Child1;
        int child2 = _nodes[index].Child2;
        _nodes[index].Bounds = BpBounds.Union(_nodes[child1].Bounds, _nodes[child2].Bounds);
        _nodes[index].Height = 1 + Math.Max(_nodes[child1].Height, _nodes[child2].Height);
    }

    private int AllocateNode()
    {
        if (_freeList == -1)
            GrowNodes();
        int index = _freeList;
        _freeList = _nodes[index].Parent;
        _nodes[index].Parent = -1;
        _nodes[index].Child1 = -1;
        _nodes[index].Child2 = -1;
        _nodes[index].Height = 0;
        _nodeCount++;
        return index;
    }

    private void FreeNode(int index)
    {
        _nodes[index].Parent = _freeList;
        _nodes[index].Height = -1;
        _nodes[index].Id = -1;
        _freeList = index;
        _nodeCount--;
    }

    private void GrowNodes()
    {
        int oldLength = _nodes.Length;
        int oldFreeList = _freeList;
        Array.Resize(ref _nodes, oldLength * 2);
        for (int i = oldLength; i < _nodes.Length - 1; i++)
        {
            _nodes[i].Parent = i + 1;
            _nodes[i].Height = -1;
        }
        _nodes[^1].Parent = oldFreeList;
        _nodes[^1].Height = -1;
        _freeList = oldLength;
    }

    private void InitializeFreeList(int start)
    {
        for (int i = start; i < _nodes.Length - 1; i++)
        {
            _nodes[i].Parent = i + 1;
            _nodes[i].Height = -1;
        }
        _nodes[^1].Parent = -1;
        _nodes[^1].Height = -1;
        _freeList = start;
    }

    private void Push(ref int count, int value)
    {
        if (count == _queryStack.Length)
            Array.Resize(ref _queryStack, _queryStack.Length * 2);
        _queryStack[count++] = value;
    }

    private void PushPair(ref int count, int a, int b)
    {
        if (count == _pairStackA.Length)
        {
            Array.Resize(ref _pairStackA, _pairStackA.Length * 2);
            Array.Resize(ref _pairStackB, _pairStackB.Length * 2);
        }
        _pairStackA[count] = a;
        _pairStackB[count] = b;
        count++;
    }

    private void ThrowIfDisposed() =>
        ObjectDisposedException.ThrowIf(Volatile.Read(ref _disposed) != 0, this);
}
