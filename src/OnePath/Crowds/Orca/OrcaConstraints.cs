using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Crowds.Orca;

/// <summary>
/// Builds ORCA half-planes (van den Berg, Guy, Lin and Manocha, "Reciprocal n-body collision avoidance", 2011,
/// sections 4 and 5.4). All positions are relative to the agent being steered, so the agent sits at the origin.
/// The construction is written down in <c>docs/orca.md</c>.
/// </summary>
public static class OrcaConstraints
{
    // A candidate normal is accepted on the boundary of the permitted-direction arc when it violates the
    // arc's inequality by at most this fraction of the scale of the problem (rounding of the leg normals).
    private const double ArcTolerance = 1e-9;

    /// <summary>
    /// The half-plane of velocities permitted to an agent A with respect to another agent B (equation 6 of the paper).
    /// </summary>
    /// <param name="relativePosition">Position of B minus position of A.</param>
    /// <param name="ownVelocity">A's optimization velocity (normally its current velocity).</param>
    /// <param name="otherVelocity">B's optimization velocity.</param>
    /// <param name="combinedRadius">Radius of A plus radius of B; at least 0.</param>
    /// <param name="timeHorizon">The look-ahead time tau; greater than 0.</param>
    /// <param name="timeStep">The simulation step, used instead of <paramref name="timeHorizon"/> while the agents overlap; greater than 0.</param>
    /// <param name="responsibility">The share of the avoidance A takes on: 0.5 when B reciprocates, 1 when B ignores A.</param>
    /// <returns>The half-plane; its normal has unit length.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="timeHorizon"/>, <paramref name="timeStep"/> or <paramref name="combinedRadius"/> is out of range.</exception>
    public static HalfPlane ForAgent(
        Vector2D relativePosition,
        Vector2D ownVelocity,
        Vector2D otherVelocity,
        double combinedRadius,
        double timeHorizon,
        double timeStep,
        double responsibility = 0.5)
    {
        Validate(combinedRadius, timeHorizon, timeStep);
        return BuildPoint(relativePosition, combinedRadius, timeHorizon, timeStep, ownVelocity - otherVelocity, responsibility, ownVelocity);
    }

    /// <summary><see cref="ForAgent"/> without the argument checks, for callers that validated the radius, horizon and step already.</summary>
    internal static HalfPlane ForAgentUnchecked(Vector2D relativePosition, Vector2D ownVelocity, Vector2D otherVelocity, double combinedRadius, double timeHorizon, double timeStep, double responsibility) =>
        BuildPoint(relativePosition, combinedRadius, timeHorizon, timeStep, ownVelocity - otherVelocity, responsibility, ownVelocity);

    /// <summary>
    /// The half-plane of velocities permitted to an agent with respect to a static line-segment obstacle (section 5.4 of the
    /// paper): the agent takes the full responsibility and its optimization velocity is zero, so the delimiting line is the tangent to the
    /// velocity obstacle at the boundary point closest to the zero velocity, and the zero velocity is always permitted
    /// while the agent does not overlap the segment.
    /// </summary>
    /// <param name="a">One end of the segment minus the agent's position.</param>
    /// <param name="b">The other end of the segment minus the agent's position; may equal <paramref name="a"/> (a point obstacle).</param>
    /// <param name="radius">The agent's radius; at least 0.</param>
    /// <param name="timeHorizon">The obstacle look-ahead time; greater than 0.</param>
    /// <param name="timeStep">The simulation step, used instead of <paramref name="timeHorizon"/> while the agent overlaps the segment; greater than 0.</param>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range.</exception>
    public static HalfPlane ForSegment(Vector2D a, Vector2D b, double radius, double timeHorizon, double timeStep)
    {
        Validate(radius, timeHorizon, timeStep);
        return Build(a, b, radius, timeHorizon, timeStep, Vector2D.Zero, 1.0, Vector2D.Zero);
    }

    /// <summary>
    /// The closest point on the boundary of the velocity obstacle of the segment <paramref name="a"/>-<paramref name="b"/> (a point obstacle
    /// when they are equal, an agent when <paramref name="a"/> is the relative position and <paramref name="radius"/> the combined radius) to the
    /// relative optimization velocity <paramref name="x"/>, as in equation 5 of the paper. The closest point is <c>x + u</c>.
    /// While the origin is within <paramref name="radius"/> of the segment the obstacle is the shape grown by the radius and cut off at
    /// <paramref name="timeStep"/>.
    /// </summary>
    /// <param name="a">One end of the segment, relative to the agent.</param>
    /// <param name="b">The other end, relative to the agent.</param>
    /// <param name="radius">The agent's radius (the combined radius for an agent).</param>
    /// <param name="timeHorizon">The look-ahead time; greater than 0.</param>
    /// <param name="timeStep">The simulation step; greater than 0.</param>
    /// <param name="x">The relative optimization velocity.</param>
    /// <param name="u">The vector from <paramref name="x"/> to the closest boundary point (zero length when <paramref name="x"/> is on the boundary).</param>
    /// <param name="normal">The unit outward normal of a supporting line of the obstacle at that point.</param>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range.</exception>
    public static void ClosestBoundaryPoint(Vector2D a, Vector2D b, double radius, double timeHorizon, double timeStep, Vector2D x, out Vector2D u, out Vector2D normal)
    {
        Validate(radius, timeHorizon, timeStep);
        bool overlapping = SegmentDistanceSquared(a, b) <= radius * radius;
        Boundary(a, b, radius, overlapping ? timeStep : timeHorizon, x, !overlapping, out normal, out double d);
        u = new Vector2D(-d * normal.X, -d * normal.Y);
    }

    /// <summary>
    /// The half-plane of velocities permitted to an agent next to one stretch of the boundary of its walkable area, where the agent's
    /// centre may touch the boundary (the area was already shrunk by the agent's radius). It is the one-sided form of section 5.4:
    /// the agent must not cross the line of the stretch within <paramref name="timeHorizon"/>, so <c>v . n &gt;= -(d - clearance) / timeHorizon</c>
    /// with <c>n</c> the unit normal pointing to the walkable side and <c>d</c> the signed distance of the agent from the line (negative when it is
    /// already across). Moving along the wall is never restricted. Only stretches the agent faces are described (one it is behind, further than 1e-6 beyond its line, is skipped): when the agent is beside the
    /// stretch, that is its projection on the line is within <paramref name="clearance"/> (plus 1e-6) of an end or beyond it, there is no half-plane, because grazing a corner is not a
    /// collision and corners are handled by the surface the agent moves on.
    /// </summary>
    /// <param name="a">Start of the stretch minus the agent's position. The walkable side is to the left of <c>a</c> to <c>b</c> (the winding of a counter-clockwise polygon).</param>
    /// <param name="b">End of the stretch minus the agent's position.</param>
    /// <param name="clearance">Distance to keep from the line; at least 0.</param>
    /// <param name="timeHorizon">Look-ahead time; greater than 0.</param>
    /// <param name="halfPlane">The constraint, when the result is true.</param>
    /// <returns>False (no constraint) when the stretch is degenerate or the agent does not project strictly inside it.</returns>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range.</exception>
    public static bool ForBoundary(Vector2D a, Vector2D b, double clearance, double timeHorizon, out HalfPlane halfPlane)
    {
        if (!(clearance >= 0 && double.IsFinite(clearance)))
        {
            throw new ArgumentOutOfRangeException(nameof(clearance), clearance, "The clearance must be finite and at least 0.");
        }
        if (!(timeHorizon > 0 && double.IsFinite(timeHorizon)))
        {
            throw new ArgumentOutOfRangeException(nameof(timeHorizon), timeHorizon, "The time horizon must be finite and greater than 0.");
        }
        halfPlane = default;
        Vector2D e = b - a;
        double ee = Vector2D.Dot(e, e);
        if (!(ee > 0))
        {
            return false;
        }
        double len = Math.Sqrt(ee);
        // Within the clearance of an end the agent is beside the stretch: a path that grazes a corner (as the taut string does) must not be
        // blocked by the wall it passes.
        double s = -Vector2D.Dot(a, e) / ee;
        double margin = (clearance + 1e-6) / len;
        if (!(s > margin && s < 1 - margin))
        {
            return false;
        }
        var n = new Vector2D(-e.Y / len, e.X / len);
        // Signed distance of the agent (the origin) from the line, positive on the walkable side.
        double d = -Vector2D.Dot(a, n);
        if (d < -1e-6)
        {
            // Behind the stretch (it faces away, as the far side of a thin wall does): it is not this agent's wall.
            return false;
        }
        halfPlane = new HalfPlane(new Vector2D(-n.X * ((d - clearance) / timeHorizon), -n.Y * ((d - clearance) / timeHorizon)), n);
        return true;
    }

    /// <summary>The squared distance from the origin to the segment <paramref name="a"/>-<paramref name="b"/>.</summary>
    public static double SegmentDistanceSquared(Vector2D a, Vector2D b)
    {
        Vector2D e = b - a;
        double ee = Vector2D.Dot(e, e);
        if (ee == 0)
        {
            return Vector2D.Dot(a, a);
        }
        double s = -Vector2D.Dot(a, e) / ee;
        s = s < 0 ? 0 : (s > 1 ? 1 : s);
        Vector2D q = a + e * s;
        return Vector2D.Dot(q, q);
    }

    private static void Validate(double radius, double tau, double dt)
    {
        if (!(radius >= 0 && double.IsFinite(radius)))
        {
            throw new ArgumentOutOfRangeException(nameof(radius), radius, "The radius must be finite and at least 0.");
        }
        if (!(tau > 0 && double.IsFinite(tau)))
        {
            throw new ArgumentOutOfRangeException(nameof(tau), tau, "The time horizon must be finite and greater than 0.");
        }
        if (!(dt > 0 && double.IsFinite(dt)))
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, "The time step must be finite and greater than 0.");
        }
    }

    // Shared by agents (a == b) and segments. x is the relative optimization velocity, origin the point the half-plane's
    // anchor is measured from, share the fraction of u that is taken.
    internal static HalfPlane Build(Vector2D a, Vector2D b, double r, double tau, double dt, Vector2D x, double share, Vector2D origin)
    {
        bool overlapping = SegmentDistanceSquared(a, b) <= r * r;
        // While overlapping, the obstacle is the shape inflated by r and cut off at one time step (see docs/orca.md).
        Boundary(a, b, r, overlapping ? dt : tau, x, !overlapping, out Vector2D n, out double d);
        // The closest boundary point is x - d*n, so u = -d*n.
        Vector2D u = new(-d * n.X, -d * n.Y);
        return new HalfPlane(origin + u * share, n);
    }

    // Build for a point obstacle (a == b), which is what another agent is. It performs exactly the operations Build and Boundary perform
    // when both ends are equal (so the result is bit for bit the same) and skips the ones that repeat a value: the second end's
    // offset, tolerance and domain test equal the first's, and the minimum of two equal dot products is that product.
    private static HalfPlane BuildPoint(Vector2D a, double r, double tau, double dt, Vector2D x, double share, Vector2D origin)
    {
        double aa = Vector2D.Dot(a, a);
        bool overlapping = aa <= r * r;
        if (!overlapping)
        {
            return Truncated(a, aa, r, tau, x, share, origin);
        }
        bool domain = !overlapping;
        double t = overlapping ? dt : tau;
        Vector2D wa = x - a / t;
        double tol = 0;
        if (domain)
        {
            tol = ArcTolerance * (a.Length() + r);
        }
        double best = double.NegativeInfinity;
        Vector2D bestN = default;
        double lenA = wa.Length();
        if (lenA > 0)
        {
            ConsiderPoint(wa / lenA, wa, domain, a, r, tol, ref best, ref bestN);
        }
        if (domain)
        {
            double l2 = aa - r * r;
            if (l2 > 0)
            {
                double l = Math.Sqrt(l2);
                double cx = -r * a.X;
                double cy = -r * a.Y;
                double px = -a.Y * l;
                double py = a.X * l;
                ConsiderPoint(new Vector2D((cx + px) / aa, (cy + py) / aa), wa, true, a, r, tol, ref best, ref bestN);
                ConsiderPoint(new Vector2D((cx - px) / aa, (cy - py) / aa), wa, true, a, r, tol, ref best, ref bestN);
            }
        }
        if (double.IsNegativeInfinity(best))
        {
            Vector2D c = a + a;
            double lc = c.Length();
            bestN = lc > 0 ? new Vector2D(-c.X / lc, -c.Y / lc) : new Vector2D(1, 0);
            best = Vector2D.Dot(bestN, wa);
        }
        double d = best - r / t;
        Vector2D u = new(-d * bestN.X, -d * bestN.Y);
        return new HalfPlane(origin + u * share, bestN);
    }

    // The truncated velocity obstacle of another agent, case by case (van den Berg et al. 2011, section 4, figure 2): the cone from the
    // origin around the direction of the agent a, cut off at time tau by the disc of radius r/tau around a/tau. The point of its boundary
    // closest to the relative velocity x is on the disc's arc (the cutoff case) or on one of the two legs (tangents from the origin).
    // Requires |a| > r. The unit normal is the obstacle's outward normal there, the vector u runs from x to that point.
    private static HalfPlane Truncated(Vector2D a, double aa, double r, double tau, Vector2D x, double share, Vector2D origin)
    {
        double wx = x.X - (a.X / tau);
        double wy = x.Y - (a.Y / tau);
        double wLenSq = (wx * wx) + (wy * wy);
        double dot1 = (wx * a.X) + (wy * a.Y);
        double nx, ny, ux, uy;
        if (dot1 < 0 && dot1 * dot1 > r * r * wLenSq)
        {
            // Cutoff case: x projects onto the arc. The outward normal is the direction from the disc's centre to x.
            double wLen = Math.Sqrt(wLenSq);
            nx = wx / wLen;
            ny = wy / wLen;
            double move = (r / tau) - wLen;
            ux = nx * move;
            uy = ny * move;
        }
        else
        {
            // Leg case: the left leg when x is counter-clockwise of the cone's axis (cross(a, w) > 0), else the right leg.
            // The leg is the ray from the origin along dir, the tangent direction of length 1 (cos = leg / |a|, sin = r / |a|).
            double leg = Math.Sqrt(aa - (r * r));
            double dx, dy;
            if ((a.X * wy) - (a.Y * wx) > 0)
            {
                dx = ((a.X * leg) - (a.Y * r)) / aa;
                dy = ((a.X * r) + (a.Y * leg)) / aa;
                nx = -dy;
                ny = dx;
            }
            else
            {
                dx = ((a.X * leg) + (a.Y * r)) / aa;
                dy = ((a.Y * leg) - (a.X * r)) / aa;
                nx = dy;
                ny = -dx;
            }
            double along = (x.X * dx) + (x.Y * dy);
            ux = (along * dx) - x.X;
            uy = (along * dy) - x.Y;
        }
        return new HalfPlane(new Vector2D(origin.X + (ux * share), origin.Y + (uy * share)), new Vector2D(nx, ny));
    }

    private static void ConsiderPoint(Vector2D n, Vector2D wa, bool domain, Vector2D a, double r, double tol, ref double best, ref Vector2D bestN)
    {
        if (domain && Vector2D.Dot(n, a) + r > tol)
        {
            return;
        }
        double g = Vector2D.Dot(n, wa);
        if (g > best)
        {
            best = g;
            bestN = n;
        }
    }

    // The signed distance from x to the boundary of S, positive outside, and the outward normal n at the closest boundary point.
    //
    // S is the velocity obstacle of the segment a-b grown by r: S = { s*k : s >= 1/tau, k in capsule(a, b, r) } (convex).
    // Its support function is h(n) = (max(n.a, n.b) + r)/tau, finite exactly for directions n with n.a <= -r and n.b <= -r
    // (the "domain", an arc of unit directions). For a convex set the signed distance is the largest over unit n of x.n - h(n):
    //     d(x) = max over n of  min_i n.(x - p_i/tau)  -  r/tau,   p_0 = a, p_1 = b.
    // The maximum of a minimum of two cosines over an arc is at one of a few candidates: the direction of either w_i = x - p_i/tau,
    // the two normals of the segment (where the two cosines cross) and the ends of the arc (the leg normals, n.p_i = -r).
    // When overlapping, S is the bounded capsule and there is no domain restriction.
    private static void Boundary(Vector2D a, Vector2D b, double r, double tau, Vector2D x, bool domain, out Vector2D normal, out double distance)
    {
        Vector2D wa = x - a / tau;
        Vector2D wb = x - b / tau;
        bool point = a.X == b.X && a.Y == b.Y;
        double tolA = 0, tolB = 0;
        if (domain)
        {
            tolA = ArcTolerance * (a.Length() + r);
            tolB = point ? tolA : ArcTolerance * (b.Length() + r);
        }

        double best = double.NegativeInfinity;
        Vector2D bestN = default;

        double lenA = wa.Length();
        if (lenA > 0)
        {
            Consider(wa / lenA, wa, wb, domain, a, b, r, tolA, tolB, ref best, ref bestN);
        }
        if (!point)
        {
            double lenB = wb.Length();
            if (lenB > 0)
            {
                Consider(wb / lenB, wa, wb, domain, a, b, r, tolA, tolB, ref best, ref bestN);
            }
            Vector2D e = b - a;
            double le = e.Length();
            if (le > 0)
            {
                Vector2D nn = new(-e.Y / le, e.X / le);
                Consider(nn, wa, wb, domain, a, b, r, tolA, tolB, ref best, ref bestN);
                Consider(-nn, wa, wb, domain, a, b, r, tolA, tolB, ref best, ref bestN);
            }
        }
        if (domain)
        {
            Legs(a, r, wa, wb, a, b, tolA, tolB, ref best, ref bestN);
            if (!point)
            {
                Legs(b, r, wa, wb, a, b, tolA, tolB, ref best, ref bestN);
            }
        }

        if (double.IsNegativeInfinity(best))
        {
            // No candidate survived (only possible through rounding at a degenerate input): face the obstacle's centre.
            Vector2D c = a + b;
            double lc = c.Length();
            bestN = lc > 0 ? new Vector2D(-c.X / lc, -c.Y / lc) : new Vector2D(1, 0);
            best = Math.Min(Vector2D.Dot(bestN, wa), Vector2D.Dot(bestN, wb));
        }
        normal = bestN;
        distance = best - r / tau;
    }

    // The two unit normals n with n.p = -r: n = (-r*p +- l*perp(p)) / |p|^2 with l = sqrt(|p|^2 - r^2), perp(p) = (-p.y, p.x).
    private static void Legs(Vector2D p, double r, Vector2D wa, Vector2D wb, Vector2D a, Vector2D b, double tolA, double tolB, ref double best, ref Vector2D bestN)
    {
        double rho2 = Vector2D.Dot(p, p);
        double l2 = rho2 - r * r;
        if (!(l2 > 0))
        {
            return;
        }
        double l = Math.Sqrt(l2);
        double cx = -r * p.X;
        double cy = -r * p.Y;
        double px = -p.Y * l;
        double py = p.X * l;
        Consider(new Vector2D((cx + px) / rho2, (cy + py) / rho2), wa, wb, true, a, b, r, tolA, tolB, ref best, ref bestN);
        Consider(new Vector2D((cx - px) / rho2, (cy - py) / rho2), wa, wb, true, a, b, r, tolA, tolB, ref best, ref bestN);
    }

    private static void Consider(Vector2D n, Vector2D wa, Vector2D wb, bool domain, Vector2D a, Vector2D b, double r, double tolA, double tolB, ref double best, ref Vector2D bestN)
    {
        if (domain && (Vector2D.Dot(n, a) + r > tolA || Vector2D.Dot(n, b) + r > tolB))
        {
            return;
        }
        double g = Math.Min(Vector2D.Dot(n, wa), Vector2D.Dot(n, wb));
        if (g > best)
        {
            best = g;
            bestN = n;
        }
    }
}
