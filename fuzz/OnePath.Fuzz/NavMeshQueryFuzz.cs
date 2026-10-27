using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target for the query layer. The bytes drive a short list of operations on a fixed multi-layer mesh with off-mesh
/// links: queries with arbitrary doubles (NaN, infinities, huge values), arbitrary 64-bit polygon references, odd extents,
/// filters and budgets, and tile removal and re-adding between queries. The only exceptions allowed are the documented
/// <see cref="ArgumentOutOfRangeException"/> for bad coordinates and extents; anything else crashes the target.
/// </summary>
internal static class NavMeshQueryFuzz
{
    private static readonly TiledNavMesh s_mesh = NavMeshWorlds.RandomMesh(21, -1, -1, 3, 3, 3, detail: true, offMesh: 2);
    private static readonly NavMeshQuery s_query = new(s_mesh, 4096);
    private static PolyRef[] s_real = Collect();

    private static PolyRef[] Collect()
    {
        var l = new System.Collections.Generic.List<PolyRef>();
        for (int s = 0; s < s_mesh.SlotCount; s++)
        {
            NavTile? t = s_mesh.GetTileBySlot(s);
            for (int p = 0; t is not null && p < t.PolyCount; p++)
            {
                l.Add(s_mesh.GetSlotRef(s).Poly(p));
            }
        }
        return l.ToArray();
    }

    private ref struct Bytes
    {
        private readonly ReadOnlySpan<byte> _b;
        private int _p;

        public Bytes(ReadOnlySpan<byte> b)
        {
            _b = b;
            _p = 0;
        }

        public bool Done => _p >= _b.Length;

        public byte U8() => _p < _b.Length ? _b[_p++] : (byte)0;

        public ulong U64()
        {
            ulong v = 0;
            for (int i = 0; i < 8; i++)
            {
                v |= (ulong)U8() << (8 * i);
            }
            return v;
        }

        // Mostly sane coordinates, sometimes the raw bit pattern (NaN, infinity, subnormal, enormous).
        public double Coord()
        {
            byte k = U8();
            if (k < 24)
            {
                return BitConverter.UInt64BitsToDouble(U64());
            }
            return -26 + ((U8() | (U8() << 8)) / 65535.0 * 28.0);
        }

        public Vector3D Vec() => new(Coord(), Coord(), Coord());

        public PolyRef Ref(bool real)
        {
            byte k = U8();
            return k < 200 || !real ? s_real[(U8() | (U8() << 8)) % s_real.Length] : new PolyRef(U64());
        }
    }

    public static void Run(byte[] data)
    {
        s_real = Collect(); // tile re-adds change slot generations
        var b = new Bytes(data);
        var corridor = new PolyRef[64];
        var points = new StraightPathPoint[64];
        var visited = new PolyRef[32];
        PolyRef removedRef = PolyRef.Invalid;
        NavTile? removed = null;
        try
        {
            for (int op = 0; op < 24 && !b.Done; op++)
            {
                var filter = new QueryFilter { ExcludeFlags = b.U8() < 40 ? (ushort)(1 << (b.U8() % 3)) : (ushort)0 };
                if (b.U8() < 60)
                {
                    filter.SetAreaCost(b.U8() % 64, 0.25 + (b.U8() / 32.0));
                }
                try
                {
                    switch (b.U8() % 8)
                    {
                        case 0:
                            _ = s_query.FindNearestPoly(b.Vec(), b.Vec(), filter);
                            break;
                        case 1:
                            _ = s_query.ClosestPointOnPoly(b.Ref(true), b.Vec(), out _, out _);
                            break;
                        case 2:
                        {
                            PathStatus s = s_query.BeginPath(b.Ref(true), b.Ref(true), b.Vec(), b.Vec(), filter);
                            int budget = 1 + (b.U8() % 40);
                            while (s == PathStatus.InProgress)
                            {
                                s = s_query.UpdatePath(budget);
                                if (b.U8() < 8)
                                {
                                    break;
                                }
                            }
                            s_query.FinishPath(corridor.AsSpan(0, 1 + (b.U8() % 64)), out int n, out _, out _);
                            if (n < 0 || n > 64)
                            {
                                throw new InvalidOperationException("corridor count out of range");
                            }
                            break;
                        }
                        case 3:
                        {
                            int n = b.U8() % 5;
                            for (int i = 0; i < n; i++)
                            {
                                corridor[i] = b.Ref(true);
                            }
                            _ = s_query.FindStraightPath(b.Vec(), b.Vec(), corridor.AsSpan(0, n), points.AsSpan(0, b.U8() % 65), out int c, out _);
                            if (c < 0 || c > 64)
                            {
                                throw new InvalidOperationException("point count out of range");
                            }
                            break;
                        }
                        case 4:
                        {
                            RaycastResult r = s_query.Raycast(b.Ref(true), b.Vec(), b.Vec(), filter, visited.AsSpan(0, b.U8() % 33), out int vc);
                            if (vc < 0 || vc > 32 || (r.Status == RaycastStatus.Ok && !(r.T >= 0 && r.T <= 1)))
                            {
                                throw new InvalidOperationException("raycast result out of range");
                            }
                            break;
                        }
                        case 5:
                        {
                            var o = new PathOptions { Filter = filter, SearchExtents = b.Vec(), MaxCorridor = b.U8(), MaxPoints = b.U8() };
                            PathResult res = s_query.FindPath(b.Vec(), b.Vec(), o);
                            if (res.HasPath && res.Points.Count == 0)
                            {
                                throw new InvalidOperationException("path without points");
                            }
                            break;
                        }
                        case 6:
                            if (removed is null)
                            {
                                PolyRef p = b.Ref(false);
                                if (s_mesh.TryGetPoly(p, out NavTile t, out _))
                                {
                                    removed = t;
                                    removedRef = p;
                                    s_mesh.RemoveTile(p.Tile);
                                }
                            }
                            else
                            {
                                s_mesh.AddTile(removed);
                                removed = null;
                            }
                            break;
                        default:
                            _ = s_query.FindNearestPoly(b.Vec(), new Vector3D(2, 4, 2), filter);
                            _ = s_query.UpdatePath(1);
                            break;
                    }
                }
                catch (ArgumentOutOfRangeException)
                {
                }
                catch (InvalidOperationException e) when (e.Message.StartsWith("The search was started with a struct", StringComparison.Ordinal))
                {
                }
            }
        }
        finally
        {
            // Leave the shared mesh as found so every input starts from the same state.
            if (removed is not null)
            {
                s_mesh.AddTile(removed);
            }
            _ = removedRef;
        }
    }
}
