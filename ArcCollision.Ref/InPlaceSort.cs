using System.Collections.Generic;

namespace ArcCollision.Ref;

/// <summary>Allocation-free deterministic heap sort for hot-path buffers.</summary>
internal static class InPlaceSort
{
    public static void Sort<T>(List<T> values, IComparer<T> comparer) =>
        Sort(ListMarshal.AsSpan(values), comparer);

    public static void Sort<T>(T[] values, int start, int count, IComparer<T> comparer) =>
        Sort(values.AsSpan(start, count), comparer);

    private static void Sort<T>(Span<T> values, IComparer<T> comparer)
    {
        int count = values.Length;
        for (int root = count / 2 - 1; root >= 0; root--)
            SiftDown(values, root, count, comparer);

        for (int end = count - 1; end > 0; end--)
        {
            (values[0], values[end]) = (values[end], values[0]);
            SiftDown(values, 0, end, comparer);
        }
    }

    private static void SiftDown<T>(
        Span<T> values, int root, int count, IComparer<T> comparer)
    {
        while (true)
        {
            int child = root * 2 + 1;
            if (child >= count) return;
            if (child + 1 < count
                && comparer.Compare(values[child], values[child + 1]) < 0)
                child++;
            if (comparer.Compare(values[root], values[child]) >= 0)
                return;
            (values[root], values[child]) = (values[child], values[root]);
            root = child;
        }
    }
}
