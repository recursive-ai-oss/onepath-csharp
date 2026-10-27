using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Crowds.Orca;

/// <summary>
/// The two-dimensional linear programs behind ORCA (paper section 5.1 and 5.3): the permitted velocity closest to the preferred
/// one inside the speed disc, and, when no velocity is permitted, the velocity that penetrates the constraints least.
/// Constraints are processed one by one in the order given (the paper's incremental method, here in a fixed order rather
/// than a random one, so the result is a function of the input list only). See <c>docs/orca.md</c>.
/// </summary>
public static class OrcaLinearProgram
{
    // |direction . normal| below this counts as parallel (both are unit vectors).
    private const double ParallelEpsilon = 1e-12;

    // Squared length below which two normals count as equal when projecting constraints for the fallback.
    private const double EqualNormalsSquared = 1e-24;

    /// <summary>
    /// Finds the velocity for an agent.
    /// </summary>
    /// <param name="constraints">Half-planes in the order they are processed. The first <paramref name="hardCount"/> are hard (obstacles).</param>
    /// <param name="hardCount">How many leading constraints are never relaxed; between 0 and the length.</param>
    /// <param name="maxSpeed">Radius of the speed disc; at least 0.</param>
    /// <param name="preferred">The preferred velocity.</param>
    /// <param name="scratch">Working space of at least <c>constraints.Length</c> half-planes (used only by the fallback).</param>
    /// <param name="feasible">True when some velocity satisfies every constraint and the disc; false when the fallback was used.</param>
    /// <returns>
    /// When feasible, the velocity inside every half-plane and the disc that is closest to <paramref name="preferred"/>.
    /// Otherwise the velocity inside the disc and inside the hard constraints (if those are feasible together; otherwise inside
    /// the disc only) that minimizes the largest penetration of the remaining constraints.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="scratch"/> is shorter than <paramref name="constraints"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="hardCount"/> or <paramref name="maxSpeed"/> is out of range.</exception>
    public static Vector2D Solve(ReadOnlySpan<HalfPlane> constraints, int hardCount, double maxSpeed, Vector2D preferred, Span<HalfPlane> scratch, out bool feasible)
    {
        if (hardCount < 0 || hardCount > constraints.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(hardCount), hardCount, "The hard count must be between 0 and the number of constraints.");
        }
        if (!(maxSpeed >= 0 && double.IsFinite(maxSpeed)))
        {
            throw new ArgumentOutOfRangeException(nameof(maxSpeed), maxSpeed, "The maximum speed must be finite and at least 0.");
        }
        if (scratch.Length < constraints.Length)
        {
            throw new ArgumentException("The scratch space must hold at least as many half-planes as there are constraints.", nameof(scratch));
        }

        int failed = Lp2(constraints, maxSpeed, preferred, false, out Vector2D v);
        if (failed < 0)
        {
            feasible = true;
            return v;
        }
        feasible = false;
        if (failed < hardCount)
        {
            // The hard constraints are not satisfiable together (cannot happen with the paper's zero optimization velocity
            // for obstacles, but a caller may build any list): relax among them and ignore the rest.
            return LeastPenetration(constraints.Slice(0, hardCount), 0, maxSpeed, scratch);
        }
        return LeastPenetration(constraints, hardCount, maxSpeed, scratch);
    }

    /// <summary>
    /// The velocity inside every half-plane and the disc of radius <paramref name="maxSpeed"/> that is closest to <paramref name="preferred"/>
    /// (or maximizes <paramref name="preferred"/> . v when <paramref name="maximize"/> is true).
    /// Returns the index of the first constraint that cannot be satisfied together with the earlier ones, or -1 on success.
    /// </summary>
    public static int Lp2(ReadOnlySpan<HalfPlane> constraints, double maxSpeed, Vector2D preferred, bool maximize, out Vector2D result)
    {
        Vector2D v;
        if (maximize)
        {
            double len = preferred.Length();
            v = len > 0 ? preferred / len * maxSpeed : Vector2D.Zero;
        }
        else
        {
            double len = preferred.Length();
            v = len > maxSpeed ? preferred / len * maxSpeed : preferred;
        }

        for (int i = 0; i < constraints.Length; i++)
        {
            if (constraints[i].SignedDistance(v) >= 0)
            {
                continue;
            }
            // The optimum so far violates constraint i, so the new optimum lies on its delimiting line.
            if (!Lp1(constraints, i, maxSpeed, preferred, maximize, ref v))
            {
                result = v;
                return i;
            }
        }
        result = v;
        return -1;
    }

    // The best point on the delimiting line of constraint i that satisfies the speed disc and constraints 0..i-1.
    private static bool Lp1(ReadOnlySpan<HalfPlane> c, int i, double maxSpeed, Vector2D objective, bool maximize, ref Vector2D v)
    {
        Vector2D p = c[i].Point;
        Vector2D n = c[i].Normal;
        Vector2D d = new(-n.Y, n.X);

        // Points p + t*d inside the disc: t^2 + 2 t (p.d) + |p|^2 - R^2 <= 0.
        double pd = Vector2D.Dot(p, d);
        double disc = pd * pd + maxSpeed * maxSpeed - Vector2D.Dot(p, p);
        if (disc < 0)
        {
            return false;
        }
        double s = Math.Sqrt(disc);
        double lo = -pd - s;
        double hi = -pd + s;

        for (int j = 0; j < i; j++)
        {
            // Constraint j along the line: (p - p_j).n_j + t (d.n_j) >= 0.
            double denom = Vector2D.Dot(d, c[j].Normal);
            double num = Vector2D.Dot(p - c[j].Point, c[j].Normal);
            if (Math.Abs(denom) <= ParallelEpsilon)
            {
                if (num < 0)
                {
                    return false;
                }
                continue;
            }
            double t = -num / denom;
            if (denom > 0)
            {
                if (t > lo)
                {
                    lo = t;
                }
            }
            else if (t < hi)
            {
                hi = t;
            }
            if (lo > hi)
            {
                return false;
            }
        }

        double tBest;
        if (maximize)
        {
            double cd = Vector2D.Dot(objective, d);
            if (cd > 0)
            {
                tBest = hi;
            }
            else if (cd < 0)
            {
                tBest = lo;
            }
            else
            {
                tBest = Clamp(Vector2D.Dot(v - p, d), lo, hi);
            }
        }
        else
        {
            tBest = Clamp(Vector2D.Dot(objective - p, d), lo, hi);
        }
        v = p + d * tBest;
        return true;
    }

    private static double Clamp(double t, double lo, double hi) => t < lo ? lo : (t > hi ? hi : t);

    // Paper section 5.3: the point (v, t) of the (v, distance) space above every constraint plane with the smallest t, found as a
    // maximum over v of the smallest signed distance g_k(v) to the constraint lines. The first hardCount constraints are kept as hard
    // half-planes. The method is incremental: when constraint j is below the current level t, the new optimum has g_j(v) = t, and for
    // each earlier soft constraint k, g_k(v) >= g_j(v) is a half-plane in v alone; the problem projects down to a 2D program
    // that maximizes g_j.
    private static Vector2D LeastPenetration(ReadOnlySpan<HalfPlane> c, int hardCount, double maxSpeed, Span<HalfPlane> scratch)
    {
        int n = c.Length;
        for (int k = 0; k < hardCount; k++)
        {
            scratch[k] = c[k];
        }
        ReadOnlySpan<HalfPlane> hard = scratch.Slice(0, hardCount);

        if (hardCount == n)
        {
            // Only the hard constraints (which are not jointly satisfiable): relax them all.
            return LeastPenetration(c, 0, maxSpeed, scratch);
        }

        // First soft constraint alone: the point of the feasible region (hard constraints and disc) deepest inside it.
        HalfPlane first = c[hardCount];
        if (Lp2(hard, maxSpeed, first.Normal, true, out Vector2D v) >= 0)
        {
            v = first.Normal * maxSpeed; // hard constraints failed through rounding; keep the disc point
        }
        double level = first.SignedDistance(v);

        for (int j = hardCount + 1; j < n; j++)
        {
            HalfPlane cj = c[j];
            if (cj.SignedDistance(v) >= level)
            {
                continue;
            }
            int count = hardCount;
            double pjn = Vector2D.Dot(cj.Point, cj.Normal);
            for (int k = hardCount; k < j; k++)
            {
                // g_k(v) >= g_j(v)  <=>  v . (n_k - n_j) >= p_k.n_k - p_j.n_j.
                Vector2D m = c[k].Normal - cj.Normal;
                double len2 = Vector2D.Dot(m, m);
                if (len2 <= EqualNormalsSquared)
                {
                    continue;
                }
                double rhs = Vector2D.Dot(c[k].Point, c[k].Normal) - pjn;
                double len = Math.Sqrt(len2);
                Vector2D mh = m / len;
                scratch[count++] = new HalfPlane(mh * (rhs / len), mh);
            }
            if (Lp2(scratch.Slice(0, count), maxSpeed, cj.Normal, true, out Vector2D vNew) < 0)
            {
                v = vNew;
                level = cj.SignedDistance(v);
            }
        }
        return v;
    }
}
