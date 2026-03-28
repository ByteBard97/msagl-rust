#!/usr/bin/env node
// api-diff.mjs — Three-way API surface comparison: C# + TS vs Rust
//
// Usage: node tools/api-diff.mjs
// Usage: node tools/api-diff.mjs --ts-only   (only show items both C# AND TS have)
// Usage: node tools/api-diff.mjs --no-toString  (suppress ToString/Compare noise)

import { readFileSync, readdirSync, statSync } from 'fs';
import { join } from 'path';

const ROOT = '/Users/ceres/Desktop/SignalCanvas';
const CS_ROUTING = join(ROOT, 'MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear');
const TS_ROUTING = join(ROOT, 'SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear');
const RUST_SRC   = join(ROOT, 'msagl-rust/src');

const ARGS = new Set(process.argv.slice(2));
const TS_ONLY    = ARGS.has('--ts-only');    // only report items confirmed in TS too
const NO_TOSTRING = ARGS.has('--no-toString'); // suppress ToString/Compare noise

// ── File mappings (C# → TS → Rust) ─────────────────────────────────────────

const MAPPINGS = [
  {
    csFile:    'RectilinearEdgeRouter.cs',
    tsFile:    'RectilinearEdgeRouter.ts',
    rustFiles: ['routing/rectilinear_edge_router.rs'],
    className: 'RectilinearEdgeRouter',
  },
  {
    csFile:    'PortManager.cs',
    tsFile:    'PortManager.ts',
    rustFiles: ['routing/port_manager.rs', 'routing/port_manager_entrances.rs', 'routing/port_manager_free_point.rs'],
    className: 'PortManager',
  },
  {
    csFile:    'TransientGraphUtility.cs',
    tsFile:    'TransientGraphUtility.ts',
    rustFiles: ['routing/transient_graph_utility.rs', 'routing/transient_graph_utility_helpers.rs', 'routing/transient_graph_utility_extend.rs'],
    className: 'TransientGraphUtility',
  },
  {
    csFile:    'StaticGraphUtility.cs',
    tsFile:    'StaticGraphUtility.ts',
    rustFiles: ['routing/static_graph_utility.rs'],
    className: 'StaticGraphUtility',
  },
  {
    csFile:    'VisibilityGraphGenerator.cs',
    tsFile:    'VisibilityGraphGenerator.ts',
    rustFiles: ['routing/visibility_graph_generator.rs', 'routing/vg_event_processing.rs', 'routing/vg_reflection_events.rs'],
    className: 'VisibilityGraphGenerator',
  },
  {
    csFile:    'FullVisibilityGraphGenerator.cs',
    tsFile:    null,
    rustFiles: ['routing/visibility_graph_generator.rs'],
    className: 'FullVisibilityGraphGenerator',
  },
  {
    csFile:    'SsstRectilinearPath.cs',
    tsFile:    'SsstRectilinearPath.ts',
    rustFiles: ['routing/path_search.rs'],
    className: 'SsstRectilinearPath',
  },
  {
    csFile:    'MsmtRectilinearPath.cs',
    tsFile:    'MsmtRectilinearPath.ts',
    rustFiles: ['routing/msmt_path.rs', 'routing/path_search_wrapper.rs'],
    className: 'MsmtRectilinearPath',
  },
  {
    csFile:    'Nudger.cs',
    tsFile:    null,
    rustFiles: ['routing/nudging/nudger.rs'],
    className: 'Nudger',
  },
  {
    csFile:    'ObstacleTree.cs',
    tsFile:    'ObstacleTree.ts',
    rustFiles: ['routing/obstacle_tree.rs', 'routing/obstacle_tree_queries.rs', 'routing/obstacle_tree_overlap.rs'],
    className: 'ObstacleTree',
  },
  {
    csFile:    'ObstaclePort.cs',
    tsFile:    'ObstaclePort.ts',
    rustFiles: ['routing/port.rs'],
    className: 'ObstaclePort',
  },
  {
    csFile:    'ObstaclePortEntrance.cs',
    tsFile:    'ObstaclePortEntrance.ts',
    rustFiles: ['routing/port_manager_entrances.rs'],
    className: 'ObstaclePortEntrance',
  },
  {
    csFile:    'ScanSegment.cs',
    tsFile:    'ScanSegment.ts',
    rustFiles: ['routing/scan_segment.rs'],
    className: 'ScanSegment',
  },
  {
    csFile:    'ScanSegmentTree.cs',
    tsFile:    'ScanSegmentTree.ts',
    rustFiles: ['routing/scan_segment.rs'],
    className: 'ScanSegmentTree',
  },
  {
    csFile:    'LookaheadScan.cs',
    tsFile:    'LookaheadScan.ts',
    rustFiles: ['routing/lookahead_scan.rs'],
    className: 'LookaheadScan',
  },
  {
    csFile:    'VertexEntry.cs',
    tsFile:    'VertexEntry.ts',
    rustFiles: ['routing/vertex_entry.rs'],
    className: 'VertexEntry',
  },
  {
    csFile:    'VisibilityVertexRectilinear.cs',
    tsFile:    'VisibilityVertexRectiline.ts',   // note TS has typo in filename
    rustFiles: ['visibility/graph.rs'],
    className: 'VisibilityVertexRectilinear',
  },
  {
    csFile:    'SpliceUtility.cs',
    tsFile:    'SpliceUtility.ts',
    rustFiles: ['routing/splice_utility.rs', 'routing/port_splice.rs'],
    className: 'SpliceUtility',
  },
  {
    csFile:    'FreePoint.cs',
    tsFile:    'FreePoint.ts',
    rustFiles: ['routing/free_point.rs'],
    className: 'FreePoint',
  },
  {
    csFile:    'Obstacle.cs',
    tsFile:    'obstacle.ts',
    rustFiles: ['routing/obstacle.rs'],
    className: 'Obstacle',
  },
  {
    csFile:    'NeighborSides.cs',
    tsFile:    'NeighborSides.ts',
    rustFiles: ['routing/neighbor_sides.rs'],
    className: 'NeighborSides',
  },
  {
    csFile:    'SegmentIntersector.cs',
    tsFile:    'SegmentIntersector.ts',
    rustFiles: ['routing/segment_intersector.rs'],
    className: 'SegmentIntersector',
  },
  {
    csFile:    'EventQueue.cs',
    tsFile:    'EventQueue.ts',
    rustFiles: ['routing/event_queue.rs'],
    className: 'EventQueue',
  },
];

// ── Noise filter ─────────────────────────────────────────────────────────────
// C# debug/comparison methods that map to Rust traits, not pub fns.
const NOISE_NAMES = new Set(['ToString', 'Compare', 'CompareTo', 'Equals', 'GetHashCode', 'Clone']);

// ── Helpers ───────────────────────────────────────────────────────────────────

function toSnakeCase(name) {
  return name
    .replace(/([A-Z]+)([A-Z][a-z])/g, '$1_$2')
    .replace(/([a-z\d])([A-Z])/g, '$1_$2')
    .toLowerCase();
}

const MODIFIER_TOKENS = new Set([
  'static', 'virtual', 'override', 'abstract', 'new', 'async',
  'internal', 'protected', 'sealed', 'readonly', 'extern', 'unsafe', 'partial',
]);

const SKIP_KEYWORDS = new Set([
  'get', 'set', 'if', 'while', 'for', 'switch', 'return', 'new', 'this', 'base',
  'operator', 'abstract', 'override', 'virtual', 'static', 'public', 'private',
  'protected', 'internal', 'async', 'sealed', 'readonly', 'const', 'event',
  'implicit', 'explicit',
]);

// ── C# extraction ─────────────────────────────────────────────────────────────

function extractCsPublicNames(filePath, className) {
  let content;
  try { content = readFileSync(filePath, 'utf8'); } catch { return []; }

  const names = new Set();
  for (const rawLine of content.split('\n')) {
    const line = rawLine.trim();
    if (!line.startsWith('public ') && !line.includes(' public ')) continue;
    if (line.startsWith('//') || line.startsWith('*') || line.startsWith('/*')) continue;
    if (line.includes(' operator ') || line.includes(' event ')) continue;

    const parenIdx = line.indexOf('(');
    const braceIdx = line.indexOf('{');
    let cutIdx = -1;
    if (parenIdx !== -1 && braceIdx !== -1) cutIdx = Math.min(parenIdx, braceIdx);
    else if (parenIdx !== -1) cutIdx = parenIdx;
    else if (braceIdx !== -1) cutIdx = braceIdx;
    if (cutIdx === -1) continue;

    const tokens = line.slice(0, cutIdx).trim().split(/\s+/);
    const pubIdx = tokens.indexOf('public');
    if (pubIdx === -1) continue;

    let i = pubIdx + 1;
    while (i < tokens.length && MODIFIER_TOKENS.has(tokens[i])) i++;

    const name = tokens[tokens.length - 1];
    if (!name || !/^[A-Z]/.test(name)) continue;
    if (name === name.toUpperCase() && name.length > 1) continue;
    if (name === className) continue;
    if (SKIP_KEYWORDS.has(name.toLowerCase())) continue;
    if (/^[A-Z]$/.test(name)) continue;
    if (name.includes('.') || name.includes(':') || name.includes('<')) continue;

    names.add(name);
  }
  return [...names];
}

// ── TS extraction ─────────────────────────────────────────────────────────────
// TS is a direct port of C# so it uses the same PascalCase names.
// We extract:
//   1. Explicit public: `public MethodName(` / `public get PropName(`
//   2. Implicit public PascalCase fields/methods at class indent level:
//      `  PascalCaseName =` or `  PascalCaseName(` or `  PascalCaseName:`

function extractTsPublicNames(filePath, className) {
  let content;
  try { content = readFileSync(filePath, 'utf8'); } catch { return []; }

  const names = new Set();
  for (const rawLine of content.split('\n')) {
    const line = rawLine.trim();
    if (line.startsWith('//') || line.startsWith('*') || line.startsWith('/*')) continue;
    if (line.includes(' operator ')) continue;

    // Explicit public keyword (same logic as C#)
    if (line.startsWith('public ')) {
      // public [get/set] Name( or public Name(
      const m = line.match(/^public\s+(?:static\s+|abstract\s+|override\s+|async\s+|get\s+|set\s+)*([A-Z]\w+)\s*[(<:=]/);
      if (m) {
        const name = m[1];
        if (name !== className && !SKIP_KEYWORDS.has(name.toLowerCase()) && !/^[A-Z]$/.test(name)) {
          names.add(name);
        }
      }
      continue;
    }

    // Implicit public: PascalCase identifier at shallow indent (class member, not local var)
    // Matches: `  PascalName =`, `  PascalName(`, `  PascalName:`, `  PascalName<`
    // Two spaces of indent = class body level
    if (/^\s{2}[A-Z]/.test(rawLine)) {
      const m = rawLine.match(/^\s{2}([A-Z]\w+)\s*[=(:;<[]/);
      if (m) {
        const name = m[1];
        if (name !== className && !SKIP_KEYWORDS.has(name.toLowerCase()) &&
            !/^[A-Z]$/.test(name) && !name.includes('.')) {
          names.add(name);
        }
      }
    }
  }
  return [...names];
}

// ── Rust extraction ───────────────────────────────────────────────────────────

// Returns a set of all pub-visible names: fns, struct fields, and type aliases.
// Struct fields are captured by tracking when we're inside a `pub struct` block
// and collecting `pub field_name:` lines.
function extractRustPubFns(filePath) {
  let content;
  try { content = readFileSync(filePath, 'utf8'); } catch { return new Set(); }

  const names = new Set();
  let braceDepth = 0;
  let inPubStruct = false;

  for (const line of content.split('\n')) {
    const trimmed = line.trim();

    // pub fn / pub(crate) fn
    const fnMatch = trimmed.match(/^pub(?:\([^)]+\))?\s+(?:async\s+)?fn\s+(\w+)\s*[(<]/);
    if (fnMatch) { names.add(fnMatch[1]); }

    // pub struct Name — start tracking its fields
    const structMatch = trimmed.match(/^pub(?:\([^)]+\))?\s+struct\s+\w+/);
    if (structMatch) { inPubStruct = true; }

    // pub type Name — type aliases (e.g. for C# properties returning complex types)
    const typeMatch = trimmed.match(/^pub(?:\([^)]+\))?\s+type\s+(\w+)/);
    if (typeMatch) { names.add(typeMatch[1]); }

    // Track brace depth to know when struct body ends
    for (const ch of trimmed) {
      if (ch === '{') braceDepth++;
      else if (ch === '}') {
        braceDepth--;
        if (braceDepth <= 0) { inPubStruct = false; braceDepth = 0; }
      }
    }

    // Inside a pub struct body: collect pub field names
    if (inPubStruct && braceDepth > 0) {
      const fieldMatch = trimmed.match(/^pub(?:\([^)]+\))?\s+(\w+)\s*:/);
      if (fieldMatch && fieldMatch[1] !== 'fn') { names.add(fieldMatch[1]); }
    }
  }
  return names;
}

function collectRsFiles(dir) {
  const results = [];
  try {
    for (const entry of readdirSync(dir)) {
      const full = join(dir, entry);
      if (statSync(full).isDirectory()) results.push(...collectRsFiles(full));
      else if (entry.endsWith('.rs')) results.push(full);
    }
  } catch { /* ignore */ }
  return results;
}

function buildGlobalRustFnSet(srcDir) {
  const all = new Set();
  for (const f of collectRsFiles(srcDir)) {
    for (const name of extractRustPubFns(f)) all.add(name);
  }
  return all;
}

// ── Main ──────────────────────────────────────────────────────────────────────

const today = new Date().toISOString().slice(0, 10);
console.log('msagl-rust API Gap Report');
console.log('C# + TS public members vs Rust pub fns');
console.log(`Generated: ${today}`);
if (TS_ONLY)    console.log('Mode: --ts-only (only items present in BOTH C# and TS)');
if (NO_TOSTRING) console.log('Mode: --no-toString (ToString/Compare suppressed)');
console.log('');

const globalRustFns = buildGlobalRustFnSet(RUST_SRC);

let totalCs = 0, totalTs = 0, totalGaps = 0, totalEntirelyMissing = 0;
const DIVIDER = '═'.repeat(72);

for (const m of MAPPINGS) {
  const csPath = m.csFile ? join(CS_ROUTING, m.csFile) : null;
  const tsPath = m.tsFile ? join(TS_ROUTING,  m.tsFile) : null;

  const csNames = csPath ? new Set(extractCsPublicNames(csPath, m.className)) : new Set();
  const tsNames = tsPath ? new Set(extractTsPublicNames(tsPath, m.className)) : new Set();

  // Union of both references — if either reference has it, we should port it
  // In --ts-only mode, only include items confirmed in TS
  const referenceNames = new Set();
  for (const n of csNames) {
    if (TS_ONLY) {
      if (tsNames.has(n)) referenceNames.add(n);
    } else {
      referenceNames.add(n);
    }
  }
  if (!TS_ONLY) {
    for (const n of tsNames) referenceNames.add(n);
  }

  // Filter noise
  if (NO_TOSTRING) {
    for (const n of NOISE_NAMES) referenceNames.delete(n);
  }

  // Rust pub fns from mapped files
  const rustFns = new Set();
  for (const rf of m.rustFiles) {
    for (const fn_ of extractRustPubFns(join(RUST_SRC, rf))) rustFns.add(fn_);
  }

  // Find gaps
  const gaps = [];
  for (const name of [...referenceNames].sort()) {
    const snake = toSnakeCase(name);
    if (rustFns.has(snake)) continue;
    const inCs = csNames.has(name);
    const inTs = tsNames.has(name);
    const existsElsewhere = globalRustFns.has(snake);
    gaps.push({ name, snake, inCs, inTs, existsElsewhere });
  }

  totalCs += csNames.size;
  totalTs += tsNames.size;
  totalGaps += gaps.length;
  totalEntirelyMissing += gaps.filter(g => !g.existsElsewhere).length;

  // Skip sections with no gaps and no reference items (uninteresting)
  if (referenceNames.size === 0 && gaps.length === 0) continue;

  console.log(DIVIDER);
  const tsLabel = m.tsFile ? m.tsFile : '(no TS file)';
  console.log(`${m.className}`);
  console.log(`  CS: ${m.csFile}  TS: ${tsLabel}`);
  console.log(`  Rust: ${m.rustFiles.join(', ')}`);
  console.log(`  Reference items — C#: ${csNames.size}  TS: ${tsNames.size}  Rust pub fns: ${rustFns.size}`);
  console.log('');

  if (gaps.length === 0) {
    console.log('  ✓ No gaps');
  } else {
    // Separate into "missing entirely" and "exists elsewhere"
    const missing  = gaps.filter(g => !g.existsElsewhere);
    const elsewhere = gaps.filter(g =>  g.existsElsewhere);

    if (missing.length > 0) {
      console.log(`  MISSING FROM src/ ENTIRELY (${missing.length}):`);
      for (const { name, snake, inCs, inTs } of missing) {
        const src = [inCs ? 'CS' : '', inTs ? 'TS' : ''].filter(Boolean).join('+');
        const pad = ' '.repeat(Math.max(0, 42 - name.length));
        console.log(`    [${src.padEnd(5)}] ${name}${pad}→  ${snake}`);
      }
    }

    if (elsewhere.length > 0) {
      console.log(`  EXISTS ELSEWHERE IN src/ (${elsewhere.length}):`);
      for (const { name, snake, inCs, inTs } of elsewhere) {
        const src = [inCs ? 'CS' : '', inTs ? 'TS' : ''].filter(Boolean).join('+');
        const pad = ' '.repeat(Math.max(0, 42 - name.length));
        console.log(`    [${src.padEnd(5)}] ${name}${pad}→  ${snake}`);
      }
    }
  }
  console.log('');
}

console.log('');
console.log('═'.repeat(54));
console.log('SUMMARY');
console.log(`  C# reference items (total):      ${totalCs}`);
console.log(`  TS reference items (total):       ${totalTs}`);
console.log(`  Gaps (not in mapped Rust files):  ${totalGaps}`);
console.log(`  Missing entirely from src/:       ${totalEntirelyMissing}`);
console.log(`  Exists elsewhere in src/:         ${totalGaps - totalEntirelyMissing}`);
console.log('');
console.log('Tips:');
console.log('  node tools/api-diff.mjs --ts-only      # only items confirmed in TS');
console.log('  node tools/api-diff.mjs --no-ToString  # hide ToString/Compare noise');
console.log('  node tools/api-diff.mjs --ts-only --no-ToString  # clean signal');
