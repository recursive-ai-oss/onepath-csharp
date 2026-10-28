namespace RecursiveAI.OnePath;

/// <summary>What one node of the polygon path search stands for.</summary>
public enum PathNodeModel
{
    /// <summary>One node per crossing (leaving a polygon through one edge or link). Exact for the midpoint-crossing cost model; about three times the nodes of <see cref="Polygon"/>, so slower.</summary>
    Crossing = 0,

    /// <summary>One node per polygon, placed at the edge midpoint where the cheapest route found so far enters it. The default: fewer nodes, so faster; the corridor may differ slightly from the crossing model's.</summary>
    Polygon = 1,
}
