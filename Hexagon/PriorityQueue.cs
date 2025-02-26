using System.Collections.Generic;
using System.Linq;

public class PriorityQueue<P, V>
{
    private SortedDictionary<P, Queue<V>> data = new SortedDictionary<P, Queue<V>>();

    public int Count => data.Count;

    public void Enqueue(P priority, V value)
    {
        Queue<V> q;
        if (!data.TryGetValue(priority, out q))
        {
            q = new Queue<V>();
            data.Add(priority, q);
        }
        q.Enqueue(value);
    }

    public V Dequeue()
    {
        // will throw if empty!
        var pair = data.First();
        var v = pair.Value.Dequeue();
        // remove top priority if empty
        if (pair.Value.Count == 0)
        {
            data.Remove(pair.Key);
        }
        
        return v;
    }

    public List<V> GetOrderedValues()
    {
        List<V> values = new List<V>(data.Count);

        if (data.Any())
        {
            foreach (var pair in data)
            {
                foreach (V value in pair.Value.ToArray())
                {
                    values.Add(value);
                }
            }
        }

        return values;
    }

    public V Peek()
    {
        var pair = data.First();
        return pair.Value.Peek();
    }

    public P FirstPriority()
    {
        return data.First().Key;
    }

    public P LastPriority()
    {
        return data.Last().Key;
    }

    public bool ContainsPriority(P priority)
    {
        return data.ContainsKey(priority);
    }

    public bool ContainsValue(V value)
    {
        foreach (KeyValuePair<P, Queue<V>> pair in data)
        {
            if (pair.Value.Contains(value))
            {
                return true;
            }
        }

        return false;
    }

    public bool IsEmpty()
    {
        return !data.Any();
    }

    public void Clear()
    {
        data.Clear();
    }
}