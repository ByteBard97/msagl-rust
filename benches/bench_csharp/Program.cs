/// <summary>
/// C# benchmark harness for MSAGL RectilinearEdgeRouter.
///
/// Loads the same JSON scenario files (small/medium/large) used by the
/// Rust, Python, and WASM benchmarks, converts them to MSAGL types,
/// runs routing 100 times, and reports mean +/- stddev in ms.
///
/// Usage: dotnet run -c Release
/// </summary>

using System.Diagnostics;
using System.Text.Json;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.Rectilinear;

const int ITERATIONS = 100;
string[] scenarioNames = { "small", "medium", "large" };

// Resolve the scenarios directory relative to this file's location.
// When run from benches/bench_csharp/, scenarios are at ../scenarios/
string scenariosDir = Path.GetFullPath(
    Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "scenarios"));

// Fallback: try relative to the working directory
if (!Directory.Exists(scenariosDir))
{
    scenariosDir = Path.GetFullPath(Path.Combine(".", "benches", "scenarios"));
}
if (!Directory.Exists(scenariosDir))
{
    scenariosDir = Path.GetFullPath(Path.Combine("..", "scenarios"));
}

Console.WriteLine("C# MSAGL Benchmark Results");
Console.WriteLine(new string('=', 60));
Console.WriteLine();

foreach (var name in scenarioNames)
{
    var scenario = LoadScenario(scenariosDir, name);
    int numObstacles = scenario.Obstacles.Length;
    int numEdges = scenario.Edges.Length;

    // ---------------------------------------------------------------
    // Correctness check (single run)
    // ---------------------------------------------------------------
    var (router, edgeGeometries) = BuildRouter(scenario);
    router.Run();

    int routed = 0;
    int unrouted = 0;
    int straight = 0;
    int bends = 0;
    foreach (var eg in edgeGeometries)
    {
        if (eg.Curve == null)
        {
            unrouted++;
            continue;
        }
        routed++;

        int waypointCount = CountWaypoints(eg);
        if (waypointCount <= 2)
        {
            straight++;
        }
        else
        {
            bends++;
        }
    }

    if (routed != numEdges)
    {
        Console.Error.WriteLine(
            $"ERROR: {name} -- expected {numEdges} routed edges, got {routed} " +
            $"({unrouted} unrouted)");
        Environment.Exit(1);
    }

    Console.WriteLine(
        $"  check: {routed}/{numEdges} routed ({straight} straight, {bends} with bends)");

    // ---------------------------------------------------------------
    // Benchmark: run ITERATIONS times
    // ---------------------------------------------------------------
    var timings = new double[ITERATIONS];
    var sw = new Stopwatch();

    for (int i = 0; i < ITERATIONS; i++)
    {
        var (r, _) = BuildRouter(scenario);

        sw.Restart();
        r.Run();
        sw.Stop();

        timings[i] = sw.Elapsed.TotalMilliseconds;
    }

    double mean = timings.Average();
    double variance = timings.Select(t => (t - mean) * (t - mean)).Sum() / ITERATIONS;
    double stddev = Math.Sqrt(variance);

    string label = $"{name,-8} ({numObstacles} obstacles, {numEdges} edges)";
    Console.WriteLine($"{label}: {mean:F3} ± {stddev:F3} ms  ({ITERATIONS} iterations)");
}

Console.WriteLine();

// =======================================================================
// Helper functions
// =======================================================================

static ScenarioData LoadScenario(string dir, string name)
{
    string path = Path.Combine(dir, $"{name}.json");
    if (!File.Exists(path))
    {
        Console.Error.WriteLine($"ERROR: scenario file not found: {path}");
        Environment.Exit(1);
    }

    string json = File.ReadAllText(path);
    var options = new JsonSerializerOptions
    {
        PropertyNameCaseInsensitive = true,
    };
    return JsonSerializer.Deserialize<ScenarioData>(json, options)
        ?? throw new InvalidOperationException($"Failed to parse {path}");
}

/// <summary>
/// Build a RectilinearEdgeRouter from a scenario, returning both the
/// router and the list of EdgeGeometry objects (for correctness checks).
/// </summary>
static (RectilinearEdgeRouter router, List<EdgeGeometry> edges) BuildRouter(
    ScenarioData scenario)
{
    // Create shapes (obstacle rectangles).
    // Our JSON format stores obstacles as {x, y, width, height} where
    // (x, y) is the top-left corner.  MSAGL's CurveFactory.CreateRectangle
    // takes (width, height, center).
    var shapes = new List<Shape>(scenario.Obstacles.Length);
    foreach (var obs in scenario.Obstacles)
    {
        double centerX = obs.X + obs.Width / 2.0;
        double centerY = obs.Y + obs.Height / 2.0;
        ICurve rect = CurveFactory.CreateRectangle(
            obs.Width, obs.Height, new Point(centerX, centerY));
        shapes.Add(new Shape(rect));
    }

    // Create the router with the scenario's padding and edge separation.
    // useSparseVisibilityGraph = false (full VG, matching our Rust port).
    // cornerFitRadius = 0 (no arc corners -- our Rust port doesn't use them).
    var router = new RectilinearEdgeRouter(
        shapes,
        padding: scenario.Padding,
        cornerFitRadius: 0.0,
        useSparseVisibilityGraph: false,
        minEdgeSeparation: scenario.EdgeSeparation);

    // Create edge geometries with FloatingPort endpoints.
    var edgeGeometries = new List<EdgeGeometry>(scenario.Edges.Length);
    foreach (var edge in scenario.Edges)
    {
        var sourceShape = shapes[edge.SourceObstacle];
        var targetShape = shapes[edge.TargetObstacle];

        var sourcePort = new FloatingPort(
            sourceShape.BoundaryCurve,
            new Point(edge.Source.X, edge.Source.Y));
        var targetPort = new FloatingPort(
            targetShape.BoundaryCurve,
            new Point(edge.Target.X, edge.Target.Y));

        // Register ports with their shapes (as the test harness does).
        sourceShape.Ports.Insert(sourcePort);
        targetShape.Ports.Insert(targetPort);

        var eg = new EdgeGeometry { SourcePort = sourcePort, TargetPort = targetPort };
        router.AddEdgeGeometryToRoute(eg);
        edgeGeometries.Add(eg);
    }

    return (router, edgeGeometries);
}

/// <summary>
/// Count the number of distinct waypoints in a routed edge's curve.
/// MSAGL produces composite Curve objects with LineSegment pieces.
/// </summary>
static int CountWaypoints(EdgeGeometry eg)
{
    if (eg.Curve == null)
    {
        return 0;
    }

    if (eg.Curve is Curve composite)
    {
        // A composite curve with N segments has N+1 unique endpoints.
        return composite.Segments.Count + 1;
    }

    if (eg.Curve is LineSegment)
    {
        return 2;
    }

    // Fallback: treat any other curve type as having 2 endpoints.
    return 2;
}

// =======================================================================
// JSON deserialization types (matches the scenario JSON schema)
// =======================================================================

public class ScenarioData
{
    public ObstacleData[] Obstacles { get; set; } = Array.Empty<ObstacleData>();
    public EdgeData[] Edges { get; set; } = Array.Empty<EdgeData>();
    public double Padding { get; set; }
    public double EdgeSeparation { get; set; }
}

public class ObstacleData
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Width { get; set; }
    public double Height { get; set; }
}

public class EdgeData
{
    public PointData Source { get; set; } = new();
    public PointData Target { get; set; } = new();
    public int SourceObstacle { get; set; }
    public int TargetObstacle { get; set; }
}

public class PointData
{
    public double X { get; set; }
    public double Y { get; set; }
}
