// tools/golden_dump/Program.cs
// Generates golden baseline JSON files by running MSAGL's RectilinearEdgeRouter
// on a set of canonical test scenarios. Outputs are used to validate the Rust port.

using System.Text.Json;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.Rectilinear;

const double DefaultPadding = 2.0;
const double DefaultCornerFitRadius = 0.0;
const double DefaultEdgeSeparation = 1.0;

var outputDir = Path.GetFullPath(
    Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "golden_baselines"));
Directory.CreateDirectory(outputDir);

Console.WriteLine($"Writing baselines to: {outputDir}");

DumpScenario("two_boxes", outputDir,
    obstacles: new[]
    {
        MakeRect(0,   0,   100, 50),
        MakeRect(300, 0,   100, 50),
    },
    edges: new[]
    {
        (0, 1, new Point(100, 25), new Point(300, 25)),
    });

DumpScenario("triangle", outputDir,
    obstacles: new[]
    {
        MakeRect(200, 0,   100, 60),
        MakeRect(0,   200, 100, 60),
        MakeRect(400, 200, 100, 60),
    },
    edges: new[]
    {
        (0, 1, new Point(250, 60),  new Point(50,  200)),
        (0, 2, new Point(250, 60),  new Point(450, 200)),
        (1, 2, new Point(100, 230), new Point(400, 230)),
    });

DumpScenario("obstacle_in_between", outputDir,
    obstacles: new[]
    {
        MakeRect(0,   25,  60, 60),  // source box
        MakeRect(300, 25,  60, 60),  // target box
        MakeRect(130, 20,  60, 60),  // blocker
    },
    edges: new[]
    {
        (0, 1, new Point(60, 55), new Point(300, 55)),
    });

DumpScenario("four_boxes_grid", outputDir,
    obstacles: new[]
    {
        MakeRect(0,   0,   60, 60),
        MakeRect(200, 0,   60, 60),
        MakeRect(0,   200, 60, 60),
        MakeRect(200, 200, 60, 60),
    },
    edges: new[]
    {
        (0, 1, new Point(60,  30),  new Point(200, 30)),
        (0, 3, new Point(30,  60),  new Point(230, 200)),
        (2, 1, new Point(60,  230), new Point(200, 30)),
    });

Console.WriteLine("Done.");
return 0;

// ---------------------------------------------------------------------------

static Shape MakeRect(double x, double y, double width, double height)
{
    var center = new Point(x + width / 2.0, y + height / 2.0);
    var curve = CurveFactory.CreateRectangle(width, height, center);
    return new Shape(curve);
}

static List<double[]> ExtractWaypoints(EdgeGeometry eg)
{
    var points = new List<double[]>();

    // SmoothedPolyline holds the un-rounded corner points of the rectilinear path.
    var sp = eg.SmoothedPolyline;
    if (sp != null)
    {
        for (var site = sp.HeadSite; site != null; site = site.Next)
            points.Add(new[] { site.Point.X, site.Point.Y });
        return points;
    }

    // Fallback: sample the composite curve segments.
    if (eg.Curve is Curve curve && curve.Segments.Count > 0)
    {
        var firstSeg = curve.Segments[0];
        points.Add(new[] { firstSeg.Start.X, firstSeg.Start.Y });
        foreach (var seg in curve.Segments)
            points.Add(new[] { seg.End.X, seg.End.Y });
        return points;
    }

    // Last resort: just endpoints.
    points.Add(new[] { eg.SourcePort.Location.X, eg.SourcePort.Location.Y });
    points.Add(new[] { eg.TargetPort.Location.X, eg.TargetPort.Location.Y });
    return points;
}

static void DumpScenario(
    string name,
    string outputDir,
    Shape[] obstacles,
    (int srcIdx, int tgtIdx, Point src, Point tgt)[] edges)
{
    Console.Write($"  {name} ... ");

    var router = new RectilinearEdgeRouter(
        obstacles,
        padding: DefaultPadding,
        cornerFitRadius: DefaultCornerFitRadius,
        useSparseVisibilityGraph: false,
        minEdgeSeparation: DefaultEdgeSeparation);

    var edgeGeometries = new List<EdgeGeometry>();
    foreach (var (srcIdx, tgtIdx, src, tgt) in edges)
    {
        var srcPort = new FloatingPort(obstacles[srcIdx].BoundaryCurve, src);
        var tgtPort = new FloatingPort(obstacles[tgtIdx].BoundaryCurve, tgt);
        var eg = new EdgeGeometry { SourcePort = srcPort, TargetPort = tgtPort };
        router.AddEdgeGeometryToRoute(eg);
        edgeGeometries.Add(eg);
    }

    router.Run();

    // Serialize obstacles as [left, bottom, right, top].
    var obstacleBoxes = obstacles.Select(s =>
    {
        var bb = s.BoundingBox;
        return new[] { bb.Left, bb.Bottom, bb.Right, bb.Top };
    }).ToArray();

    // Serialize each routed path.
    var paths = edgeGeometries.Select((eg, i) =>
    {
        var (srcIdx, tgtIdx, src, tgt) = edges[i];
        return new
        {
            src_obstacle = srcIdx,
            tgt_obstacle = tgtIdx,
            src_port = new[] { src.X, src.Y },
            tgt_port = new[] { tgt.X, tgt.Y },
            waypoints = ExtractWaypoints(eg),
            routed = eg.Curve != null,
        };
    }).ToArray();

    var result = new
    {
        scenario = name,
        padding = DefaultPadding,
        corner_fit_radius = DefaultCornerFitRadius,
        edge_separation = DefaultEdgeSeparation,
        obstacles = obstacleBoxes,
        paths,
    };

    var options = new JsonSerializerOptions { WriteIndented = true };
    var json = JsonSerializer.Serialize(result, options);
    File.WriteAllText(Path.Combine(outputDir, name + ".json"), json);
    Console.WriteLine($"ok ({paths.Length} edge(s))");
}
