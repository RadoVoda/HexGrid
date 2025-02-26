using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;
using BurstEnums;
using UnityEngine.Pool;

namespace Hexagon
{
    /// <summary>
    /// Struct representing hexagonal grid coordinate
    /// </summary>
    public readonly struct Hex : IEquatable<Hex>, IComparable<Hex>
    {
        #region Fields

        public readonly int x;
        public readonly int y;
        public readonly int z;

        /// <summary>
        /// Default hex periodic sequence
        /// </summary>
        public const int DEFAULT_SEQUENCE = 3;

        /// <summary>
        /// Width in 2D coordinates
        /// </summary>
        public int w => x;

        /// <summary>
        /// Height in 2D coordinates
        /// </summary>
        public int h => y + ((x - (x & 1)) >> 1);

        /// <summary>
        /// None represents an invalid hex coordinate
        /// Any hex where x + y + z != 0 is invalid,
        /// but constructors ensure that all invalid
        /// coords are conflated to this one for simplicity
        /// </summary>
        public static readonly Hex None = new Hex(0, 0, -1);

        /// <summary>
        /// Zero is equal to the coordinates of default hex constructor
        /// </summary>
        public static readonly Hex Zero = new Hex(0, 0, 0);

        public static readonly Hex[] emptyHexArray = new Hex[0];
        public static readonly Hex[,] emptyHexArray2D = new Hex[0,0];
        public static readonly Hex.EDirection[] emptyHexDirectionArray = new EDirection[0];

        #endregion

        #region Enums

        [Flags]
        public enum EDirection : byte
        {
            North = 1 << 0,
            NorthEast = 1 << 1,
            SouthEast = 1 << 2,
            South = 1 << 3,
            SouthWest = 1 << 4,
            NorthWest = 1 << 5
        }

        [Flags]
        public enum ESegment : byte
        {
            NorthEast = 1 << 0,
            East = 1 << 1,
            SouthEast = 1 << 2,
            SouthWest = 1 << 3,
            West = 1 << 4,
            NorthWest = 1 << 5
        }

        #endregion

        #region Constructors

        /// <summary>
        /// 2D hex coordinate constructor
        /// Remember that all 2D hex coordinates are valid and thus invalid Hex.None coordinate cannot be created
        /// </summary>
        public Hex(int width, int height)
        {
            x = width;
            y = height - ((x - (x & 1)) >> 1);
            z = -x - y;
        }

        /// <summary>
        /// 3D hex coordinate constructor
        /// Uses bitwise magic to ensure that any invalid coordinates will create the same Hex.None coordinate (0,0,-1) without any branching checks
        /// </summary>
        public Hex(int X, int Y, int Z)
        {
            int mask = -Branchless.IsZero(X + Y + Z);
            x = X & mask;
            y = Y & mask;
            z = Z & mask;
            z += ~mask;
        }

        #endregion

        #region Functions

        /// <summary>
        /// Get array of hex neighbours
        /// </summary>
        public Hex[] GetNeighbours()
        {
            if (!IsValid())
                return emptyHexArray;

            Hex[] neighbours = new Hex[6];

            neighbours[0] = new Hex(x, y + 1, z - 1);
            neighbours[1] = new Hex(x + 1, y, z - 1);
            neighbours[2] = new Hex(x + 1, y - 1, z);
            neighbours[3] = new Hex(x, y - 1, z + 1);
            neighbours[4] = new Hex(x - 1, y, z + 1);
            neighbours[5] = new Hex(x - 1, y + 1, z);

            return neighbours;
        }

        /// <summary>
        /// Get enumerable struct of hex neighbours, does not allocate managed memory
        /// </summary>
        public HexNeighbours Neighbours => new HexNeighbours(this);

        /// <summary>
        /// Get array of hex diagonals
        /// </summary>
        public Hex[] GetDiagonals()
        {
            if (!IsValid())
                return emptyHexArray;

            Hex[] diagonals = new Hex[6];

            diagonals[0] = new Hex(x + 1, y + 1, z - 2);
            diagonals[1] = new Hex(x + 2, y - 1, z - 1);
            diagonals[2] = new Hex(x + 1, y - 2, z + 1);
            diagonals[3] = new Hex(x - 1, y - 1, z + 2);
            diagonals[4] = new Hex(x - 2, y + 1, z + 1);
            diagonals[5] = new Hex(x - 1, y + 2, z - 1);

            return diagonals;
        }

        /// <summary>
        /// Get enumerable struct of hex diagonals, does not allocate managed memory
        /// </summary>
        public HexDiagonals Diagonals => new HexDiagonals(this);

        /// <summary>
        /// Get array of Nth neighbours at N distance
        /// </summary>
        public Hex[] GetSequenceNeighbours(int s = DEFAULT_SEQUENCE)
        {
            if (!IsValid())
                return emptyHexArray;

            if (!IsSequenceHex(s))
                return GetClosestSequenceHex(s).GetSequenceNeighbours(s);

            Hex[] neighbours = new Hex[6];

            neighbours[0] = new Hex(x, y + s, z - s);
            neighbours[1] = new Hex(x + s, y, z - s);
            neighbours[2] = new Hex(x + s, y - s, z);
            neighbours[3] = new Hex(x, y - s, z + s);
            neighbours[4] = new Hex(x - s, y, z + s);
            neighbours[5] = new Hex(x - s, y + s, z);

            return neighbours;
        }

        /// <summary>
        /// Get enumerable struct of Nth neighbours at N distance, does not allocate managed memory
        /// </summary>
        public HexSequenceNeighbours SequenceNeighbours(int s = DEFAULT_SEQUENCE)
        {
            return IsSequenceHex(s) || !IsValid() ?
                new HexSequenceNeighbours(this, s) :
                GetClosestSequenceHex(s).SequenceNeighbours(s);
        }

        /// <summary>
        /// Get array of all hexes within given radius
        /// </summary>
        public Hex[] GetHexesInRadius(int radius)
        {
            if (!IsValid() || radius < 0)
                return emptyHexArray;

            int fill = 0;
            int area = 3 * radius * (radius + 1) + 1;
            Hex[] hexes = new Hex[area];

            if (radius == 0)
            {
                hexes[0] = this;
                return hexes;
            }

            for (int x = -radius; x <= radius; ++x)
            {
                for (int y = Math.Max(-radius, -x - radius); y <= Math.Min(radius, -x + radius); ++y)
                {
                    hexes[fill++] = this + new Hex(x, y, (-x - y));
                }
            }

            return hexes;
        }

        /// <summary>
        /// Get enumerable struct of all hexes within given radius, does not allocate managed memory
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HexRadius HexesInRadius(int radius) => new HexRadius(this, radius);

        /// <summary>
        /// Get enumerable struct of all hexes within given segment, does not allocate managed memory
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HexSegment HexesInSegment(int range, ESegment segments) => new HexSegment(this, range, segments);

        /// <summary>
        /// Get segment in which target hex is positioned relative to the origin hex
        /// </summary>
        public ESegment GetRelativeHexSegment(in Hex target)
        {
            if (!IsValid() || !target.IsValid()) return 0;
            if (target.x > x && target.y <= y && target.z <= z) return ESegment.East;
            if (target.x >= x && target.y >= y && target.z < z) return ESegment.NorthEast;
            if (target.x >= x && target.y < y && target.z >= z) return ESegment.SouthEast;
            if (target.x <= x && target.y > y && target.z <= z) return ESegment.NorthWest;
            if (target.x <= x && target.y <= y && target.z > z) return ESegment.SouthWest;
            if (target.x < x && target.y >= y && target.z >= z) return ESegment.West;
            return 0;
        }

        /// <summary>
        /// Get array of all hexes that are exactly at given range, a ring of hexes at given distance
        /// </summary>
        public Hex[] GetHexesAtRange(int range)
        {
            if (!IsValid() || range <= 0)
                return emptyHexArray;

            int fill = 0;
            int ring = range * 6;
            Hex[] hexes = new Hex[ring];
            Hex hex = GetHexAtDirectionAndRange(EDirection.SouthWest, range);

            for (int i = 1; i <= 32; i *= 2)
            {
                for (int j = 0; j < range; ++j)
                {
                    hexes[fill++] = hex;
                    hex = hex.GetHexAtDirection((EDirection)i);
                }
            }

            return hexes;
        }

        /// <summary>
        /// Get enumerable struct of all hexes that are exactly at given range,
        /// effectively a ring of hexes at given distance, does not allocate managed memory
        /// </summary>
        public HexRing HexesAtRange(int range) => new HexRing(this, range);

        /// <summary>
        /// Get 2D array of all hexes within 2D offset coordinate range
        /// </summary>
        public Hex[,] GetHexesInOffset(int offset)
        {
            if (!IsValid() || offset <= 0)
                return emptyHexArray2D;

            int width = w;
            int height = h;
            int size = 1 + (offset << 1);
            bool odd = !IsEven();
            Hex[,] hexes = new Hex[size, size];

            for (int w = -offset; w <= offset; ++w)
            {
                Branchless.binary binary = !(odd || w.IsEven());

                for (int h = -offset; h <= offset; ++h)
                {
                    hexes[w + offset, h + offset] = new Hex(width + w, height + h - binary);
                }
            }

            return hexes;
        }

        /// <summary>
        /// Get enumarable struct of all hexes within 2D offset coordinate range, does not allocate managed memory
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HexOffset HexesInOffset(int offset) => new HexOffset(this, offset);

        /// <summary>
        /// Get array of all hexes that are on a straight line between this and given hex, including both end hexes
        /// </summary>
        public Hex[] GetHexesInLine(in Hex hex)
        {
            int fill = 0;
            int distance = Distance(hex) + 1;
            Hex[] line = new Hex[distance];
            Hex delta = hex - this;

            while (fill < distance)
            {
                line[fill++] = hex - delta;
                delta = Reduce(delta);
            }

            return line;
        }

        /// <summary>
        /// Get enumerable struct of all hexes that are on a straight line between this and given hex,
        /// including both end hexes, does not allocate managed memory
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HexLine HexesInLine(in Hex to) => new HexLine(this, to);

        /// <summary>
        /// Reduce hex magnitude by one, i.e return hex one closer to Hex.Zero.
        /// Achieved by reducing two of the Hex x,y,z coordinates with the highest
        /// absolute values by one so that the total sum remains zero, i.e valid.
        /// Attempt to reduce Hex.Zero will return Hex.Zero.
        /// Attempt to reduce Hex.None will return Hex.None.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Hex Reduce(in Hex c)
        {
            int valid = Branchless.IsZero(c.x + c.y + c.z);
            int xAbs = c.x.Abs();
            int yAbs = c.y.Abs();
            int zAbs = c.z.Abs();
            int xVSy = xAbs.IsGreaterOrEqualThan(yAbs);
            int yVSz = yAbs.IsGreaterOrEqualThan(zAbs);
            int zVSx = zAbs.IsGreaterOrEqualThan(xAbs);
            int xScl = (xVSy | (1 - zVSx)) & valid;
            int yScl = (yVSz | (1 - xVSy)) & valid;
            int zScl = (zVSx | (1 - yVSz)) & valid;
            return new Hex(c.x.Scale(-xScl), c.y.Scale(-yScl), c.z.Scale(-zScl));
        }

        /// <summary>
        /// Get closest Nth hex to this hex
        /// </summary>
        public Hex GetClosestSequenceHex(int s = DEFAULT_SEQUENCE)
        {
            if (!IsValid() || IsSequenceHex(s))
                return this;

            return new Hex(x.GetClosestDivisibleBy(s), y.GetClosestDivisibleBy(s), z.GetClosestDivisibleBy(s));
        }

        /// <summary>
        /// Get hex at given direction and range
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Hex GetHexAtDirectionAndRange(EDirection direction, int range = 1)
        {
            if (IsValid() && range > 0)
            {
                switch (direction)
                {
                    case EDirection.North: return new Hex(x, y + range, z - range);
                    case EDirection.NorthEast: return new Hex(x + range, y, z - range);
                    case EDirection.SouthEast: return new Hex(x + range, y - range, z);
                    case EDirection.South: return new Hex(x, y - range, z + range);
                    case EDirection.SouthWest: return new Hex(x - range, y, z + range);
                    case EDirection.NorthWest: return new Hex(x - range, y + range, z);
                    default: return None;
                }
            }

            return this;
        }

        /// <summary>
        /// Get closest key hex at given direction and range
        /// </summary>
        public Hex GetNthHexAtDirectionAndRange(EDirection direction, int range, int n = DEFAULT_SEQUENCE)
        {
            if (!IsValid() || !direction.IsValid())
                return this;

            if (!IsSequenceHex(n))
                GetClosestSequenceHex(n).GetNthHexAtDirectionAndRange(direction, range, n);

            if (range > 0)
            {
                range *= 3;

                switch (direction)
                {
                    case EDirection.North: return new Hex(x, y + range, z - range);
                    case EDirection.NorthEast: return new Hex(x + range, y, z - range);
                    case EDirection.SouthEast: return new Hex(x + range, y - range, z);
                    case EDirection.South: return new Hex(x, y - range, z + range);
                    case EDirection.SouthWest: return new Hex(x - range, y, z + range);
                    case EDirection.NorthWest: return new Hex(x - range, y + range, z);
                    default: return None;
                }
            }

            return None;
        }

        /// <summary>
        /// Get diagonal hex that is within given segment
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Hex GetDiagonalInSegment(ESegment segment)
        {
            if (IsValid())
            {
                switch (segment)
                {
                    case ESegment.NorthEast: return new Hex(x + 1, y + 1, z - 2);
                    case ESegment.East: return new Hex(x + 2, y - 1, z - 1);
                    case ESegment.SouthEast: return new Hex(x + 1, y - 2, z + 1);
                    case ESegment.SouthWest: return new Hex(x - 1, y - 1, z + 2);
                    case ESegment.West: return new Hex(x - 2, y + 1, z + 1);
                    case ESegment.NorthWest: return new Hex(x - 1, y + 2, z - 1);
                    default: return None;
                }
            }

            return None;
        }

        /// <summary>
        /// Get neighbour hex at given direction
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Hex GetHexAtDirection(EDirection direction) => GetHexAtDirectionAndRange(direction, 1);

        /// <summary>
        /// Get closest key hex at given direction
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Hex GetNthHexAtDirection(EDirection direction, int n = DEFAULT_SEQUENCE)
        {
            return GetClosestSequenceHex(n).GetHexAtDirectionAndRange(direction, n);
        }

        /// <summary>
        /// Get direction to the given neighbour hex or zero if hex is not a neighbour hex
        /// </summary>
        public EDirection GetDirectionToNeighbourHex(in Hex hex)
        {
            if (Distance(hex) == 1)
            {
                if (x == hex.x && y == hex.y - 1 && z == hex.z + 1) return EDirection.North;
                if (x == hex.x - 1 && y == hex.y && z == hex.z + 1) return EDirection.NorthEast;
                if (x == hex.x - 1 && y == hex.y + 1 && z == hex.z) return EDirection.SouthEast;
                if (x == hex.x && y == hex.y + 1 && z == hex.z - 1) return EDirection.South;
                if (x == hex.x + 1 && y == hex.y && z == hex.z - 1) return EDirection.SouthWest;
                if (x == hex.x + 1 && y == hex.y - 1 && z == hex.z) return EDirection.NorthWest;
            }

            return 0;
        }

        /// <summary>
        /// Get direction to the given key hex
        /// </summary>
        public EDirection GetDirectionOfNthHex(Hex hex, int n = DEFAULT_SEQUENCE)
        {
            if (IsValid() && hex.IsValid())
            {
                if (!IsSequenceHex(n))
                    return GetClosestSequenceHex(n).GetDirectionOfNthHex(hex, n);

                if (!hex.IsSequenceHex(n))
                    hex = hex.GetClosestSequenceHex(n);

                if (IsSequenceNeighbour(hex, n))
                {
                    if (x == hex.x && y == hex.y - n && z == hex.z + n) return EDirection.North;
                    if (x == hex.x - n && y == hex.y && z == hex.z + n) return EDirection.NorthEast;
                    if (x == hex.x - n && y == hex.y + n && z == hex.z) return EDirection.SouthEast;
                    if (x == hex.x && y == hex.y + n && z == hex.z - n) return EDirection.South;
                    if (x == hex.x + n && y == hex.y && z == hex.z - n) return EDirection.SouthWest;
                    if (x == hex.x + n && y == hex.y - n && z == hex.z) return EDirection.NorthWest;
                }
            }

            return 0;
        }

        /// <summary>
        /// Get list of all hexes that are within layout reconstructed from 64 bit array
        /// </summary>
        public List<Hex> GetLayoutFromBitArray(BitArray64 data)
        {
            int count = data.BitCount();
            const int size = 7;
            List<Hex> layout = new(count);

            if (!IsValid() || count == 0 || size == 0)
                return layout;

            var offset = HexesInOffset(3);

            for (int i = 0; i < size; i++)
            {
                for (int j = 0; j < size; j++)
                {
                    if (data[i, j])
                    {
                        count--;
                        layout.Add(offset[i, j]);

                        if (count == 0)
                        {
                            return layout;
                        }
                    }
                }
            }

            return layout;
        }

        /// <summary>
        /// Get enumerable readonly struct of all hexes that are within layout reconstructed from 64 bit array
        /// </summary>
        public HexLayout GetHexLayout(BitArray64 bits) => new HexLayout(this, bits);

        /// <summary>
        /// Rotate hex layout given amount of times, positive rotation count are clockwise, negative are anti-clockwise
        /// </summary>
        public void RotateHexLayout(Hex[,] layout, int rotation)
        {
            bool clockwise = rotation > 0;
            int rotations = Math.Abs(rotation) % 6;

            if (IsValid() && layout != null && rotation != 0)
            {
                for (int i = 0; i < layout.GetLength(0); ++i)
                {
                    for (int j = 0; j < layout.GetLength(0); ++j)
                    {
                        Hex hex = layout[i, j];

                        if (hex.IsValid())
                        {
                            Hex vector = hex - this;

                            for (int r = 0; r < rotations; ++r)
                            {
                                vector = vector.Rotate(clockwise);
                            }

                            layout[i, j] = vector + this;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Rotate target hex around this hex given amount of times, positive rotation count are clockwise, negative are anti-clockwise
        /// </summary>
        public Hex RotateHex(in Hex hex, int rotation)
        {
            if (!IsValid() || !hex.IsValid())
                return None;

            bool clockwise = rotation > 0;
            int rotations = math.abs(rotation) % 6;

            if (rotation == 0)
                return hex;

            Hex vector = hex - this;

            for (int r = 0; r < rotations; ++r)
            {
                vector = vector.Rotate(clockwise);
            }

            return vector + this;
        }

        /// <summary>
        /// Return hex world position from size offsets, as hex itself has no idea of the world scale
        /// </summary>
        public static Vector3 GetHexWorldPosition(in Hex hex, float3 offsets)
        {
            if (!hex.IsValid())
                return Vector3.zero;

            float width = hex.w * offsets.x * 0.75f;
            float height = hex.h * offsets.y;
            float depth = hex.h * offsets.z;

            if (!hex.IsEven())
            {
                height += offsets.y * 0.5f;
                depth += offsets.z * 0.5f;
            }

            return new Vector3(width, height, depth);
        }

        /// <summary>
        /// Return hex at given world position with given hex offsets
        /// </summary>
        public static Hex GetHexAtWorldPosition(Vector2 position, float3 offsets)
        {
            int x = (int)Math.Round(position.x / offsets.x / 0.75f);

            if (!x.IsEven())
                position.y -= offsets.y * 0.5f;

            int y = (int)Math.Round(position.y / offsets.y);
            return new Hex(x, y);
        }

        /// <summary>
        /// Get array of vectors that will outline given hex when connected with consecutive line based on size offsets
        /// </summary>
        public static Vector3[] GetHexOutline(in Hex hex, float3 offsets)
        {
            if (!hex.IsValid())
                return new Vector3[0];

            Vector3 position = GetHexWorldPosition(hex, offsets);
            Vector3[] outline = new Vector3[6];
            outline[0] = new Vector3(position.x + offsets.x * 0.25f, position.y + offsets.y * 0.5f, 0);
            outline[1] = new Vector3(position.x + offsets.x * 0.5f, position.y, 0);
            outline[2] = new Vector3(position.x + offsets.x * 0.25f, position.y - offsets.y * 0.5f, 0);
            outline[3] = new Vector3(position.x - offsets.x * 0.25f, position.y - offsets.y * 0.5f, 0);
            outline[4] = new Vector3(position.x - offsets.x * 0.5f, position.y, 0);
            outline[5] = new Vector3(position.x - offsets.x * 0.25f, position.y + offsets.y * 0.5f, 0);
            return outline;
        }

        /// <summary>
        /// Get list of consecutive border hexes of a given hex layout
        /// </summary>
        public static List<Hex> GetHexPerimeter(IReadOnlyList<Hex> hexes)
        {
            if (hexes.Count == 0)
                return new List<Hex>(0);

            HashSet<Hex> matchSet = new HashSet<Hex>(hexes);//initialized from collection so cannot be taken from pool
            Dictionary<Hex, int> neighbourCounts = new Dictionary<Hex, int>();
            Hex start = None;

            foreach (Hex hex in hexes)
            {
                if (hex.IsValid() == false)
                    continue;

                if (start.IsValid() == false)
                    start = hex;

                var neighbours = hex.Neighbours;
                int neighbourCount = 0;

                foreach (Hex neighbour in neighbours)
                {
                    if (neighbour.IsValid() && matchSet.Contains(neighbour))
                        neighbourCount++;
                }

                neighbourCounts[hex] = neighbourCount;

                if (neighbourCounts.TryGetValue(start, out int startNeighbours) && neighbourCount < startNeighbours ||
                    hex.w + hex.h > start.w + start.h)
                {
                    start = hex;
                }
            }

            List<Hex> perimeter = new List<Hex>(hexes.Count);
            EDirection direction = EDirection.NorthEast;
            Hex last = start;

            do
            {
                var searchDirection = direction.GetPerimeterSearchDirections();

                foreach (EDirection search in searchDirection)
                {
                    Hex next = last.GetHexAtDirection(search);

                    if (matchSet.Contains(next))
                    {
                        perimeter.Add(last);
                        direction = search;
                        last = next;
                        break;
                    }
                }
            }
            while (last != start);

            perimeter.Add(start);
            return perimeter;
        }

        /// <summary>
        /// Get path between two hexes on the given cost map. Zero or negative hex cost value is an obstacle
        /// </summary>
        public static List<Hex> Pathfinding(in Hex start, in Hex stop, Dictionary<Hex, int> moveCostMap)
        {
            int distance = start.Distance(stop);
            List<Hex> pathSet = new List<Hex>(distance);
            PriorityQueue<int, Hex> openSet = new PriorityQueue<int, Hex>();
            HashSet<Hex> doneSet = new HashSet<Hex>(distance);
            Dictionary<Hex, Hex> fromSet = new Dictionary<Hex, Hex>(distance);
            Dictionary<Hex, int> costSet = new Dictionary<Hex, int>(distance);

            openSet.Enqueue(0, start);
            costSet.Add(start, 0);

            while (!openSet.IsEmpty())
            {
                Hex current = openSet.Dequeue();

                if (current == stop)
                {
                    pathSet.Add(current);

                    while (fromSet.ContainsKey(current))
                    {
                        fromSet.TryGetValue(current, out current);
                        pathSet.Add(current);
                    }

                    return pathSet;
                }

                int oldCost = costSet[current];

                foreach (Hex neighbour in current.Neighbours)
                {
                    if (moveCostMap.TryGetValue(neighbour, out int cost) &&
                        cost > 0 &&
                        !doneSet.Contains(neighbour) &&
                        !costSet.ContainsKey(neighbour))
                    {
                        int newCost = oldCost + cost;
                        costSet.Add(neighbour, newCost);
                        int priority = newCost + stop.Distance(neighbour);
                        openSet.Enqueue(priority, neighbour);
                        fromSet.Add(neighbour, current);
                    }
                }

                doneSet.Add(current);
            }

            return pathSet;
        }

        /// <summary>
        /// Rotate hex coordinates given amount of times, positive rotation count are clockwise, negative are anti-clockwise
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Hex Rotate(bool clockwise) => clockwise ? new Hex(-z, -x, -y) : new Hex(-y, -z, -x);

        /// <summary>
        /// Check if target hex is neighbour of this one
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsNeighbour(in Hex hex) => Distance(hex) == 1;

        /// <summary>
        /// Check if target hex is key neighbour of this one
        /// </summary>

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsSequenceNeighbour(in Hex hex, int n = DEFAULT_SEQUENCE) => IsSequenceHex(n) && hex.IsSequenceHex(n) && Distance(hex) == n;

        /// <summary>
        /// Returns actual distance between two hexes
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Distance(in Hex hex) => IsValid() && hex.IsValid() ? math.max(math.max(math.abs(x - hex.x), math.abs(y - hex.y)), math.abs(z - hex.z)) : -1;

        /// <summary>
        /// Returns 2D coordinate distance between two hexes, i.e NOT the actual hex count between them,
        /// but rather how far in Width and height offsets they are apart.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int OffsetDistance(in Hex hex) => IsValid() && hex.IsValid() ? math.max(math.abs(w - hex.w), math.abs(h - hex.h)) : -1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => $"Hex [W:{w} H:{h} X:{x} Y:{y} Z:{z} ]";

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object other) => other is Hex hex && Equals(hex);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Hex other) => x == other.x && y == other.y && z == other.z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsEven() => (x & 1) == 0;

        /// <summary>
        /// Any hex that does not fulfill this criteria is not a valid hex grid coordinate
        /// Because Hex constructors enforce that all invalid coordinates will always return the same
        /// Hex.None coordinate it is perfectly possible to use "someHex != Hex.None" as well.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsValid() => x + y + z == 0;

        /// <summary>
        /// Check if this hex is a at N periodic sequence from zero
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsSequenceHex(int s = DEFAULT_SEQUENCE) => IsValid() && x % s == 0 && y % s == 0;

        /// <summary>
        /// Hex hash code is build on assumption that most hexes in actual use have Width and Height coordinates
        /// within short type value range, i.e between -32,768 to 32,767. Thus if we encode the Width coordinate
        /// to the first 16 bits and Height coordinate to last 16 bits we are guaranteed that the most
        /// frequently used hex coordinates will all have unique 32 bit integer hash values. This also means
        /// that if all hexes in use are strictly within this range they can be represented by a single integer.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode() => (w << 16) ^ (h & 0xffff);

        /// <summary>
        /// If all hexes in actual use do have their Width and Height coordinates strictly
        /// within short type value range, i.e between -32,768 to 32,767 they are guaranteed
        /// to have unique 32 bit integer hash values and so can be reconstructed from hash.
        /// This will not work correctly for any hex coodinates outside of this range!
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Hex FromHashCode(int hash)
        {
            int w = hash >> 16;
            int h = hash << 16 >> 16;
            return new Hex(w, h);
        }

        /// <summary>
        /// Hexes are sorted by their distance from zero
        /// using x > y > z coordinates to determine ties.
        /// </summary>
        public int CompareTo(Hex other)
        {
            if (this == other) return 0;
            int thisDist = this.Distance(Zero);
            int otherDist = other.Distance(Zero);
            if (thisDist < otherDist) return -1;
            if (thisDist > otherDist) return 1;
            if (x < other.x) return -1;
            if (x > other.x) return 1;
            if (y < other.y) return -1;
            if (y > other.y) return 1;
            if (z < other.z) return -1;
            if (z > other.z) return 1;
            return 0;
        }

        #endregion

        #region Operators

        /* One thing to remember when converting hex to different value types is that a hex can be represented
         * in both 2d and 3d coordinate systems. However in 2d plane all hex coordinates are valid and thus
         * an invalid hex cannot be represented at all. Converting an invalid Hex.None to 2d will return
         * a valid Hex.Zero coodinate. Therefore, if you need to use invalid hex always stick to 3d coordinates,
         * but if you are sure that your hex coordinates are valid you can freely convert them between 2d and 3d,
         * for example when you need to store a lot of them and want to save space.
         */
        public static implicit operator Vector3Int(Hex value) => new Vector3Int(value.x, value.y, value.z);
        public static implicit operator Hex(Vector3Int value) => new Hex(value.x, value.y, value.z);
        public static implicit operator Vector2Int(Hex value) => new Vector2Int(value.w, value.h);
        public static implicit operator Hex(Vector2Int value) => new Hex(value.x, value.y);
        public static implicit operator int3(Hex value) => new int3(value.x, value.y, value.z);
        public static implicit operator Hex(int3 value) => new Hex(value.x, value.y, value.z);
        public static implicit operator int2(Hex value) => new int2(value.w, value.h);
        public static implicit operator Hex(int2 value) => new Hex(value.x, value.y);
        public static implicit operator bool(Hex value) => value.IsValid();
        public static Hex operator +(in Hex a, in Hex b) => a.IsValid() && b.IsValid() ? new Hex(a.x + b.x, a.y + b.y, a.z + b.z) : None;
        public static Hex operator -(in Hex a, in Hex b) => a.IsValid() && b.IsValid() ? new Hex(a.x - b.x, a.y - b.y, a.z - b.z) : None;
        public static bool operator ==(Hex a, Hex b) => a.x == b.x && a.y == b.y && a.z == b.z;
        public static bool operator !=(Hex a, Hex b) => !(a == b);

        #endregion

        #region RefStructs

        /// <summary>
        /// Struct representing a collection of hex neighbours without allocating any managed memory
        /// </summary>
        public ref struct HexNeighbours
        {
            public readonly Hex hex;
            private Hex current;
            private int direction;
            public const int length = 6;

            public HexNeighbours(in Hex hex)
            {
                this.hex = hex;
                direction = 1;
                current = None;
            }

            public Hex this[EDirection d] => hex.GetHexAtDirection(d);
            public Hex this[int i] => i >= 0 && i < length ? hex.GetHexAtDirection((EDirection)(1 << i)) : None;

            public bool MoveNext()
            {
                current = hex.GetHexAtDirection((EDirection)direction);
                direction <<= 1;
                return current.IsValid() && direction <= (1 << length);
            }

            public void Reset()
            {
                direction = 1;
                current = None;
            }

            public bool TryDequeue(out Hex next)
            {
                bool move = MoveNext();
                next = current;
                return move;
            }

            public bool Contains(in Hex hex)
            {
                foreach (var item in GetEnumerator())
                {
                    if (item == hex)
                        return true;
                }

                return false;
            }

            public HexNeighbours GetEnumerator() => new HexNeighbours(hex);
            public Hex Current { get { return current; } }
            public int Count { get { return hex.IsValid() ? length : 0; } }
        }

        /// <summary>
        /// Struct representing a collection of hex neighbours at n distance without allocating any managed memory
        /// </summary>
        public ref struct HexSequenceNeighbours
        {
            public readonly Hex hex;
            private Hex current;
            private int direction;
            private readonly int sequence;
            public const int length = 6;

            public HexSequenceNeighbours(in Hex hex, int sequence = DEFAULT_SEQUENCE)
            {
                this.hex = hex;
                direction = 1;
                current = None;
                this.sequence = sequence;
            }

            public Hex this[EDirection d] => hex.GetNthHexAtDirection(d, sequence);
            public Hex this[int i] => i >= 0 && i < length ? hex.GetNthHexAtDirection((EDirection)(1 << i), sequence) : None;

            public bool MoveNext()
            {
                current = hex.GetNthHexAtDirection((EDirection)direction, sequence);
                direction <<= 1;
                return current.IsValid() && direction <= (1 << length);
            }

            public void Reset()
            {
                direction = 1;
                current = None;
            }

            public bool TryDequeue(out Hex next)
            {
                bool move = MoveNext();
                next = current;
                return move;
            }

            public bool Contains(in Hex hex)
            {
                foreach (var item in GetEnumerator())
                {
                    if (item == hex)
                        return true;
                }

                return false;
            }

            public HexSequenceNeighbours GetEnumerator() => new HexSequenceNeighbours(hex, sequence);
            public Hex Current { get { return current; } }
            public int Count { get { return hex.IsValid() ? length : 0; } }
        }

        /// <summary>
        /// Struct representing a collection of hex diagonal neighbours without allocating any managed memory
        /// </summary>
        public ref struct HexDiagonals
        {
            public readonly Hex hex;
            private Hex current;
            private int segment;
            public const int length = 6;

            public HexDiagonals(in Hex hex)
            {
                this.hex = hex;
                segment = 1;
                current = None;
            }

            public Hex this[ESegment s] => hex.GetDiagonalInSegment(s);
            public Hex this[int i] => i >= 0 && i < length ? hex.GetDiagonalInSegment((ESegment)(1 << i)) : None;

            public bool MoveNext()
            {
                current = hex.GetDiagonalInSegment((ESegment)segment);
                segment <<= 1;
                return current.IsValid() && segment < (1 << length);
            }

            public void Reset()
            {
                segment = 1;
                current = None;
            }

            public bool TryDequeue(out Hex next)
            {
                bool move = MoveNext();
                next = current;
                return move;
            }

            public bool Contains(in Hex hex)
            {
                foreach (var item in GetEnumerator())
                {
                    if (item == hex)
                        return true;
                }

                return false;
            }

            public HexDiagonals GetEnumerator() => new HexDiagonals(hex);
            public Hex Current { get { return current; } }
            public int Count { get { return hex.IsValid() ? length : 0; } }
        }

        /// <summary>
        /// Struct representing a collection of a continuous line of hexes without allocating any managed memory
        /// </summary>
        public ref struct HexLine
        {
            public readonly Hex from;
            public readonly Hex to;
            private Hex delta;
            private Hex current;

            public HexLine(in Hex from, in Hex to)
            {
                this.from = from;
                this.to = to;
                delta = to - from;
                current = None;
            }

            public Hex this[int i]
            {
                get
                {
                    if (i >= 0 && i < Count)
                    {
                        Hex d = to - from;

                        while (i > 0)
                        {
                            d = Reduce(d);
                            i--;
                        }

                        return to - d;
                    }

                    return None;
                }
            }

            public bool MoveNext()
            {
                if (delta != None)
                {
                    current = to - delta;
                    delta = delta != Zero ? Reduce(delta) : None;
                    return current.IsValid();
                }

                return false;
            }

            public void Reset()
            {
                delta = to - from;
                current = None;
            }

            public bool TryDequeue(out Hex next)
            {
                bool move = MoveNext();
                next = current;
                return move;
            }

            public bool Contains(in Hex hex)
            {
                foreach (var item in GetEnumerator())
                {
                    if (item == hex)
                        return true;
                }

                return false;
            }

            public HexLine GetEnumerator() => new HexLine(to, from);
            public Hex Current { get { return current; } }
            public int Count { get { return to.Distance(from) + 1; } }
        }

        /// <summary>
        /// Struct representing a collection of a all hexes within certain radius without allocating any managed memory
        /// </summary>
        public ref struct HexRadius
        {
            public readonly Hex hex;
            private Hex current;
            public readonly int radius;
            private int x, y;

            public HexRadius(in Hex hex, int radius)
            {
                this.hex = hex;
                this.radius = radius;
                x = -radius;
                y = math.max(-radius, -x - radius);
                current = None;
            }

            public Hex this[int i]
            {
                get
                {
                    if (i >= 0 && i <= Count)
                    {
                        for (int x = -radius; x <= radius; ++x)
                        {
                            for (int y = math.max(-radius, -x - radius); y <= math.min(radius, -x + radius); ++y)
                            {
                                if (i == 0)
                                {
                                    return hex + new Hex(x, y, (-x - y));
                                }

                                i--;
                            }
                        }
                    }

                    return None;
                }
            }

            public bool MoveNext()
            {
                if (y <= math.min(radius, -x + radius))
                {
                    current = hex + new Hex(x, y, -x - y);
                    y++;
                    return current.IsValid();
                }
                else if (x < radius)
                {
                    x++;
                    y = math.max(-radius, -x - radius);
                    current = hex + new Hex(x, y, -x - y);
                    y++;
                    return current.IsValid();
                }
                else return false;
            }

            public void Reset()
            {
                x = -radius;
                y = math.max(-radius, -x - radius);
                current = None;
            }

            public bool TryDequeue(out Hex next)
            {
                bool move = MoveNext();
                next = current;
                return move;
            }

            public bool Contains(in Hex hex)
            {
                foreach (var item in GetEnumerator())
                {
                    if (item == hex)
                        return true;
                }

                return false;
            }

            public HexRadius GetEnumerator() => new HexRadius(hex, radius);
            public Hex Current { get { return current; } }
            public int Count { get { return 3 * radius * (radius + 1) + 1; } }
        }

        /// <summary>
        /// Struct representing a collection of a all hexes in a ring at a certain distance without allocating any managed memory
        /// </summary>
        public ref struct HexRing
        {
            public readonly Hex hex;
            private Hex current;
            public readonly int radius;
            private int d, r;

            public HexRing(in Hex hex, int radius)
            {
                this.hex = hex;
                this.radius = radius;
                d = 1;
                r = 0;
                current = None;
            }

            public Hex this[int i]
            {
                get
                {
                    if (i >= 0 && i < Count)
                    {
                        Hex temp = hex.GetHexAtDirectionAndRange(EDirection.SouthWest, radius);

                        for (int d = 1; d <= 32; d <<= 1)
                        {
                            for (int r = 0; r < radius; ++r)
                            {
                                if (i > 0)
                                {
                                    i--;
                                    temp = temp.GetHexAtDirection((EDirection)d);
                                }
                                else
                                {
                                    return temp;
                                }

                            }
                        }
                    }

                    return None;
                }
            }

            public bool MoveNext()
            {
                while (d <= 32)
                {
                    while (r < radius)
                    {
                        r++;
                        current = current.GetHexAtDirection((EDirection)d);
                        return current.IsValid();
                    }

                    d <<= 1;
                    r = 0;
                }

                return false;
            }

            public void Reset()
            {
                d = 1;
                r = 0;
                current = None;
            }

            public bool TryDequeue(out Hex next)
            {
                bool move = MoveNext();
                next = current;
                return move;
            }

            public bool Contains(in Hex hex)
            {
                foreach (var item in GetEnumerator())
                {
                    if (item == hex)
                        return true;
                }

                return false;
            }

            public HexOffset GetEnumerator() => new HexOffset(hex, radius);
            public Hex Current { get { return current; } }
            public int Count { get { return radius * 6; } }
        }

        /// <summary>
        /// Struct representing a collection of a all hexes within directional segment area without allocating any managed memory
        /// </summary>
        public ref struct HexSegment
        {
            private HexRadius radius;
            private Hex current;
            public readonly ESegment segments;
            public readonly int area;

            public HexSegment(in Hex hex, int range, ESegment segments)
            {
                radius = new HexRadius(hex, range);
                this.segments = segments;
                area = 1;
                current = None;
                int value = (int)segments;
                int size = ((range + 1) * (range + 2) >> 1) - 1;
                bool last = (value & 32) != 0;
                //reduce area for overlapping arcs
                for (int s = 1; s <= value; s <<= 1)
                {
                    if ((value & s) != 0)
                    {
                        area += last ? size - range : size;
                        last = true;
                    }
                    else last = false;
                }
            }

            public Hex this[int i]
            {
                get
                {
                    if (i >= 0 && i < area)
                    {
                        var tempRadius = new HexRadius(radius.hex, radius.radius);

                        while (tempRadius.TryDequeue(out var hexInRadius))
                        {
                            if ((segments & tempRadius.hex.GetRelativeHexSegment(hexInRadius)) != 0)
                            {
                                if (i == 0)
                                {
                                    return hexInRadius;
                                }

                                i--;
                            }
                        }
                    }

                    return None;
                }
            }

            public bool MoveNext()
            {
                while (radius.TryDequeue(out var hexInRadius))
                {
                    if ((segments & radius.hex.GetRelativeHexSegment(hexInRadius)) != 0)
                    {
                        current = hexInRadius;
                        return current.IsValid();
                    }
                }

                return false;
            }

            public void Reset()
            {
                radius.Reset();
                current = None;
            }

            public bool TryDequeue(out Hex next)
            {
                bool move = MoveNext();
                next = current;
                return move;
            }

            public bool Contains(in Hex hex)
            {
                foreach (var item in GetEnumerator())
                {
                    if (item == hex)
                        return true;
                }

                return false;
            }

            public HexSegment GetEnumerator() => new HexSegment(radius.hex, radius.radius, segments);
            public Hex Current { get { return current; } }
            public int Count { get { return area; } }
        }

        /// <summary>
        /// Struct representing a collection of a 2D coordinate grid of all hexes within certain Width and Height,
        /// without allocating any managed memory
        /// </summary>
        public ref struct HexOffset
        {
            public readonly Hex hex;
            private Hex current;
            public readonly int offset;
            private int w, h;

            public HexOffset(in Hex hex, int offset)
            {
                this.hex = hex;
                this.offset = offset;
                w = -offset;
                h = -offset;
                current = None;
            }

            public Hex this[int i]
            {
                get
                {
                    int length = (1 + (offset << 1));

                    if (i >= 0 && i < length * length)
                    {
                        int Width = Math.DivRem(i, length, out int Height);
                        return this[Width, Height];
                    }

                    return None;
                }
            }

            public Hex this[int width, int height]
            {
                get
                {
                    width -= offset;
                    height -= offset;

                    if (width >= -offset && width <= offset && height >= -offset && height <= offset)
                    {
                        Branchless.binary binary = !(!hex.IsEven() || width.IsEven());
                        return new Hex(hex.w + width, hex.h + height - binary);
                    }

                    return None;
                }
            }

            public bool MoveNext()
            {
                if (h <= offset)
                {
                    Branchless.binary binary = !(!hex.IsEven() || w.IsEven());
                    current = new Hex(hex.w + w, hex.h + h - binary);
                    h++;
                    return current.IsValid();
                }

                if (w < offset)
                {
                    w++;
                    h = -offset;
                    Branchless.binary binary = !(!hex.IsEven() || w.IsEven());
                    current = new Hex(hex.w + w, hex.h + h - binary);
                    h++;
                    return current.IsValid();
                }

                return false;
            }

            public void Reset()
            {
                w = -offset;
                h = -offset;
                current = None;
            }

            public bool TryDequeue(out Hex next)
            {
                bool move = MoveNext();
                next = current;
                return move;
            }

            public bool Contains(in Hex hex)
            {
                foreach (var item in GetEnumerator())
                {
                    if (item == hex)
                        return true;
                }

                return false;
            }

            public HexOffset GetEnumerator() => new HexOffset(hex, offset);
            public Hex Current { get { return current; } }
            public int Count { get { return (1 + (offset << 1)) * (1 + (offset << 1)); } }
        }

        /// <summary>
        /// Struct representing a collection of selected hexes within 7x7 grid without allocating any managed memory
        /// </summary>
        public ref struct HexLayout
        {
            private readonly Hex hex;
            private Hex current;
            private readonly BitArray64 bits;
            private int w, h;
            public const int size = 7;

            public HexLayout(in Hex hex, BitArray64 bits)
            {
                this.hex = hex;
                this.bits = bits;
                w = 0;
                h = 0;
                current = None;
            }

            public Hex this[int i]
            {
                get
                {
                    if (i >= 0 && i < Count)
                    {
                        for (int width = 0; width < size; ++width)
                        {
                            for (int height = 0; height < size; ++height)
                            {
                                if (bits[width, height])
                                {
                                    if (i == 0)
                                    {
                                        Branchless.binary binary = !(!hex.IsEven() || !width.IsEven());
                                        return new Hex(hex.w + width - 3, hex.h + height - 3 - binary);
                                    }

                                    i--;
                                }
                            }
                        }
                    }

                    return None;
                }
            }

            public bool MoveNext()
            {
                while (w < size)
                {
                    while (h < size)
                    {
                        if (bits[w, h])
                        {
                            Branchless.binary binary = !(!hex.IsEven() || !w.IsEven());
                            current = new Hex(hex.w + w - 3, hex.h + h - 3 - binary);
                            h++;
                            return current.IsValid();
                        }

                        h++;
                    }

                    w++;
                    h = 0;
                }

                return false;
            }

            public void Reset()
            {
                w = 0;
                h = 0;
                current = None;
            }

            public bool TryDequeue(out Hex next)
            {
                bool move = MoveNext();
                next = current;
                return move;
            }

            public bool Contains(in Hex hex)
            {
                foreach (var item in GetEnumerator())
                {
                    if (item == hex)
                        return true;
                }

                return false;
            }

            public HexLayout GetEnumerator() => new HexLayout(hex, bits);
            public Hex Current { get { return current; } }
            public int Count { get { return bits.BitCount(); } }
        }

        #endregion
    }

    #region Extensions

    public static partial class Extensions
    {
        /// <summary>
        /// Return opposite direction/s
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Hex.EDirection Opposite(this Hex.EDirection direction)
        {
            if (!direction.IsValid())
                return 0;

            var flags = direction.BitFlags();

            Hex.EDirection opposite = 0;

            foreach (Hex.EDirection flag in flags)
            {
                opposite |= flag.Rotate(3);
            }

            return opposite;
        }

        /// <summary>
        /// Rotate direction enum given amount of times, positive rotation count are clockwise, negative are anti-clockwise
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Hex.EDirection Rotate(this Hex.EDirection direction, int rotation)
        {
            rotation += math.tzcnt(direction.ToMask());//add direction to rotation as final rotation is counted from 1
            rotation %= 6;//there are six rotations of the hexagon so translate higher rotations to this interval by getting a remainder
            rotation = rotation.IsPositive().IfElse(rotation, rotation + 6);//remap all rotation to clockwise
            rotation = 1 << rotation;//count final rotation from 1
            return (Hex.EDirection)rotation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Hex.EDirection[] GetPerimeterSearchDirections(this Hex.EDirection direction)
        {
            if (!direction.IsValid())
                return Hex.emptyHexDirectionArray;

            Hex.EDirection[] directions = new Hex.EDirection[6];
            directions[0] = direction.Rotate(-1);
            directions[1] = direction;
            directions[2] = direction.Rotate(1);
            directions[3] = direction.Rotate(2);
            directions[4] = direction.Rotate(3);
            directions[5] = direction.Rotate(4);
            return directions;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsSide(this Hex.EDirection direction) => direction.IsValid() && direction != Hex.EDirection.North && direction != Hex.EDirection.South;
    }

    #endregion
}