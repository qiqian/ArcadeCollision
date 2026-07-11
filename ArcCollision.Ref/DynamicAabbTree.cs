using System;
using System.Collections.Generic;

namespace ArcCollision;

internal sealed class DynamicAabbTree
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
    private readonly Dictionary<int, int> _proxies = new();
    private int[] _queryStack = new int[64];
    private int _root = -1;
    private int _freeList;
    private int _nodeCount;

    public DynamicAabbTree() => InitializeFreeList(0);

    public int Count => _proxies.Count;

    public void Clear()
    {
        _proxies.Clear();
        _root = -1;
        _nodeCount = 0;
        InitializeFreeList(0);
    }

    public void Insert(int id, in BpBounds fatBounds)
    {
        if (_proxies.ContainsKey(id))
            throw new ArgumentException($"Dynamic proxy id {id} already exists.", nameof(id));

        int leaf = AllocateNode();
        _nodes[leaf].Bounds = fatBounds;
        _nodes[leaf].Id = id;
        _nodes[leaf].Height = 0;
        _nodes[leaf].Child1 = -1;
        _nodes[leaf].Child2 = -1;
        _proxies.Add(id, leaf);
        InsertLeaf(leaf);
    }

    public bool Move(int id, in BpBounds bounds, in BpBounds fatBounds)
    {
        int leaf = _proxies[id];
        if (_nodes[leaf].Bounds.Contains(bounds))
            return false;

        RemoveLeaf(leaf);
        _nodes[leaf].Bounds = fatBounds;
        InsertLeaf(leaf);
        return true;
    }

    public bool Remove(int id)
    {
        if (!_proxies.Remove(id, out int leaf))
            return false;
        RemoveLeaf(leaf);
        FreeNode(leaf);
        return true;
    }

    public void Query(in BpBounds bounds, List<int> results)
    {
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
        Array.Resize(ref _nodes, oldLength * 2);
        InitializeFreeList(oldLength);
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
}
