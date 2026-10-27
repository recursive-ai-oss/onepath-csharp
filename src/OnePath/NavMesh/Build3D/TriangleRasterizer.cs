using System;

namespace RecursiveAI.OnePath.NavMesh.Build3D;

/// <summary>
/// Clips one triangle against the grid and emits spans. Coordinates are scaled to cell units (u = x / cellSize,
/// v = y / cellHeight, w = z / cellSize), so every clip plane sits at an integer. Clipping is Sutherland-Hodgman
/// splitting of a convex polygon by an axis-aligned plane (Sutherland &amp; Hodgman 1974); points exactly on the plane
/// belong to both halves, so pieces are closed sets and the tie rules in docs/build3d-spans.md can be stated on them.
/// </summary>
internal sealed class TriangleRasterizer
{
    private struct P
    {
        public float U;
        public float V;
        public float W;
    }

    private const int Cap = 32;

    private readonly SolidHeightfield _hf;
    private readonly int _width;
    private readonly int _depth;
    private readonly int _height;
    private readonly float _cs;
    private readonly float _ch;
    private readonly int _threshold;

    private P[] _a = new P[Cap];
    private P[] _b = new P[Cap];
    private P[] _rest = new P[Cap];
    private P[] _tmp = new P[Cap];
    private P[] _row = new P[Cap];
    private P[] _cell = new P[Cap];
    private readonly P[] _crest = new P[Cap];
    private readonly P[] _ctmp = new P[Cap];

    private float _ny;
    private byte _area;
    private bool _touched;
    private bool _lineMode;

    internal TriangleRasterizer(SolidHeightfield hf, int threshold)
    {
        _hf = hf;
        _width = hf.Frame.Width;
        _depth = hf.Frame.Depth;
        _height = hf.Frame.Height;
        _cs = hf.Frame.CellSize;
        _ch = hf.Frame.CellHeight;
        _threshold = threshold;
    }

    /// <summary>Rasterizes one non-degenerate triangle given in tile-local float32. Returns true when it added at least one span.</summary>
    internal bool Triangle(float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2, float nx, float ny, float nz, byte area)
    {
        _ny = ny;
        _area = area;
        _touched = false;
        P[] poly = _a;
        poly[0] = new P { U = x0 / _cs, V = y0 / _ch, W = z0 / _cs };
        poly[1] = new P { U = x1 / _cs, V = y1 / _ch, W = z1 / _cs };
        poly[2] = new P { U = x2 / _cs, V = y2 / _ch, W = z2 / _cs };
        float u0 = poly[0].U, w0 = poly[0].W, u1 = poly[1].U, w1 = poly[1].W, u2 = poly[2].U, w2 = poly[2].W;

        // Vertical triangle: its footprint has zero area, so it is rasterized as a line.
        float footprint = ((u1 - u0) * (w2 - w0)) - ((u2 - u0) * (w1 - w0));
        _lineMode = footprint == 0f;
        int fixedCol = -1;
        int fixedRow = -1;
        if (_lineMode)
        {
            if (u0 == u1 && u1 == u2 && IsInteger(u0))
            {
                // Lies exactly on a grid line: belongs to the cell behind the surface (opposite its normal).
                fixedCol = ClampToInt(nx > 0f ? u0 - 1f : u0, _width);
                if (fixedCol < 0)
                {
                    return false;
                }
            }
            else if (w0 == w1 && w1 == w2 && IsInteger(w0))
            {
                fixedRow = ClampToInt(nz > 0f ? w0 - 1f : w0, _depth);
                if (fixedRow < 0)
                {
                    return false;
                }
            }
        }

        // Restrict to the grid rectangle [0, W] x [0, D].
        int n = 3;
        n = Clip(poly, n, 0, 0f, false, _b);
        n = Clip(_b, n, 0, _width, true, _a);
        n = Clip(_a, n, 2, 0f, false, _b);
        n = Clip(_b, n, 2, _depth, true, _a);
        if (n == 0)
        {
            return false;
        }
        poly = _a;

        float minW = poly[0].W;
        float maxW = minW;
        for (int i = 1; i < n; i++)
        {
            minW = Math.Min(minW, poly[i].W);
            maxW = Math.Max(maxW, poly[i].W);
        }
        int j0 = fixedRow >= 0 ? fixedRow : Math.Clamp((int)Math.Floor(minW), 0, _depth - 1);
        int j1 = fixedRow >= 0 ? fixedRow : Math.Clamp((int)Math.Ceiling(maxW) - 1, j0, _depth - 1);

        Array.Copy(poly, _rest, n);
        int restN = n;
        for (int j = j0; j <= j1; j++)
        {
            int rowN;
            if (j < j1)
            {
                rowN = Clip(_rest, restN, 2, j + 1, true, _row);
                restN = Clip(_rest, restN, 2, j + 1, false, _tmp);
                (_rest, _tmp) = (_tmp, _rest);
            }
            else
            {
                Array.Copy(_rest, _row, restN);
                rowN = restN;
            }
            if (rowN == 0 || !Positive(_row, rowN))
            {
                continue;
            }
            RowToCells(rowN, j, fixedCol);
        }
        return _touched;
    }

    private void RowToCells(int rowN, int j, int fixedCol)
    {
        if (fixedCol >= 0)
        {
            Array.Copy(_row, _cell, rowN);
            Emit(fixedCol, j, rowN);
            return;
        }
        float minU = _row[0].U;
        float maxU = minU;
        for (int i = 1; i < rowN; i++)
        {
            minU = Math.Min(minU, _row[i].U);
            maxU = Math.Max(maxU, _row[i].U);
        }
        int i0 = Math.Clamp((int)Math.Floor(minU), 0, _width - 1);
        int i1 = Math.Clamp((int)Math.Ceiling(maxU) - 1, i0, _width - 1);
        // Walk the row left to right, splitting off one cell at a time.
        P[] rest = _crest;
        P[] spare = _ctmp;
        Array.Copy(_row, rest, rowN);
        int restN = rowN;
        for (int i = i0; i <= i1; i++)
        {
            int cellN;
            if (i < i1)
            {
                cellN = Clip(rest, restN, 0, i + 1, true, _cell);
                restN = Clip(rest, restN, 0, i + 1, false, spare);
                (rest, spare) = (spare, rest);
            }
            else
            {
                Array.Copy(rest, _cell, restN);
                cellN = restN;
            }
            if (cellN == 0 || !Positive(_cell, cellN))
            {
                continue;
            }
            Emit(i, j, cellN);
        }
    }

    private void Emit(int i, int j, int n)
    {
        float vmin = _cell[0].V;
        float vmax = vmin;
        for (int k = 1; k < n; k++)
        {
            vmin = Math.Min(vmin, _cell[k].V);
            vmax = Math.Max(vmax, _cell[k].V);
        }
        double lo = Math.Floor((double)vmin);
        double hi = Math.Ceiling((double)vmax);
        if (lo == hi)
        {
            // A horizontal piece exactly on a row boundary occupies the row on the solid side: below an
            // upward-facing surface, above a downward-facing one.
            if (_ny > 0f)
            {
                lo = hi - 1.0;
            }
            else
            {
                hi = lo + 1.0;
            }
        }
        if (hi <= 0.0 || lo >= _height)
        {
            return;
        }
        int smin = lo < 0.0 ? 0 : (int)lo;
        int smax = hi > _height ? _height : (int)hi;
        _hf.AddSpan(i + (j * _width), smin, smax, _area, _threshold);
        _touched = true;
    }

    // A piece counts when it has positive area (positive length for vertical triangles).
    private bool Positive(P[] p, int n)
    {
        if (n < 2)
        {
            return false;
        }
        if (_lineMode)
        {
            float minU = p[0].U, maxU = minU, minW = p[0].W, maxW = minW;
            for (int i = 1; i < n; i++)
            {
                minU = Math.Min(minU, p[i].U);
                maxU = Math.Max(maxU, p[i].U);
                minW = Math.Min(minW, p[i].W);
                maxW = Math.Max(maxW, p[i].W);
            }
            return maxU > minU || maxW > minW;
        }
        if (n < 3)
        {
            return false;
        }
        // Fan of cross products about vertex 0: pieces whose points share an axis-aligned line sum to exactly 0.
        float sum = 0f;
        float bu = p[0].U;
        float bw = p[0].W;
        for (int i = 1; i + 1 < n; i++)
        {
            sum += ((p[i].U - bu) * (p[i + 1].W - bw)) - ((p[i + 1].U - bu) * (p[i].W - bw));
        }
        return sum != 0f;
    }

    private static bool IsInteger(float v) => v == Math.Floor(v);

    // Cell index for an integer-valued coordinate, or -1 when it is outside [0, limit).
    private static int ClampToInt(float v, int limit) => v >= 0f && v < limit ? (int)v : -1;

    private static float Get(in P p, int axis) => axis == 0 ? p.U : p.W;

    // Keeps the part of the convex polygon with axis <= c (low) or axis >= c (not low). Points on the plane are kept.
    private static int Clip(P[] src, int n, int axis, float c, bool low, P[] dst)
    {
        if (n == 0)
        {
            return 0;
        }
        int m = 0;
        P prev = src[n - 1];
        float dp = Get(prev, axis) - c;
        for (int i = 0; i < n; i++)
        {
            P cur = src[i];
            float dc = Get(cur, axis) - c;
            if ((dp < 0f && dc > 0f) || (dp > 0f && dc < 0f))
            {
                float t = dp / (dp - dc);
                P x = new P
                {
                    U = prev.U + ((cur.U - prev.U) * t),
                    V = prev.V + ((cur.V - prev.V) * t),
                    W = prev.W + ((cur.W - prev.W) * t),
                };
                if (axis == 0)
                {
                    x.U = c;
                }
                else
                {
                    x.W = c;
                }
                dst[m++] = x;
            }
            if (low ? dc <= 0f : dc >= 0f)
            {
                dst[m++] = cur;
            }
            prev = cur;
            dp = dc;
        }
        return m;
    }
}
