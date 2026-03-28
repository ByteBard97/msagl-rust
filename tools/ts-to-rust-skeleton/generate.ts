#!/usr/bin/env npx ts-node
/**
 * TS-to-Rust skeleton generator.
 *
 * Reads a TypeScript reference excerpt and emits a Rust skeleton file with:
 * - Method signatures with todo!() bodies
 * - Per-method comments: TS line range, line count, branch structure, key calls
 * - Positive directive comments for data structure usage
 *
 * Usage:
 *   npx ts-node generate.ts <input.ts> <output.rs> [--struct StructName] [--crate-prefix prefix]
 *
 * Example:
 *   npx ts-node generate.ts ../../reference/port_manager_entrances.ts /dev/stdout --struct FullPortManager
 */

import * as ts from "typescript";
import * as fs from "fs";
import * as path from "path";

// ── Type mapping table (CLAUDE.md rule 2) ────────────────────────────────

const TYPE_MAP: Record<string, string> = {
  number: "f64",
  boolean: "bool",
  string: "String",
  void: "()",
  Point: "Point",
  Rectangle: "Rectangle",
  Direction: "CompassDirection",
  VisibilityVertex: "VertexId",
  VisibilityEdge: "(VertexId, VertexId, f64)",
  VisibilityGraph: "VisibilityGraph",
  ObstaclePort: "ObstaclePort",
  ObstaclePortEntrance: "ObstaclePortEntrance",
  ObstacleTree: "ObstacleTree",
  Obstacle: "Obstacle",
  FreePoint: "FreePoint",
  ScanSegment: "ScanSegment",
  ScanSegmentTree: "ScanSegmentTree",
  ScanDirection: "ScanDirection",
  TransientGraphUtility: "TransientGraphUtility",
  ICurve: "Polyline",
  LineSegment: "(Point, Point)",
  Shape: "Shape",
  Port: "Point",
  GeomEdge: "(Point, Point)",
};

function mapType(tsType: string): string {
  // Strip whitespace
  tsType = tsType.trim();

  // Array<T> or T[]
  const arrayMatch = tsType.match(/^Array<(.+)>$/) || tsType.match(/^(.+)\[\]$/);
  if (arrayMatch) {
    return `Vec<${mapType(arrayMatch[1])}>`;
  }

  // Set<T>
  const setMatch = tsType.match(/^Set<(.+)>$/);
  if (setMatch) {
    return `HashSet<${mapType(setMatch[1])}>`;
  }

  // Map<K, V>
  const mapMatch = tsType.match(/^Map<(.+),\s*(.+)>$/);
  if (mapMatch) {
    return `HashMap<${mapType(mapMatch[1])}, ${mapType(mapMatch[2])}>`;
  }

  // PointMap<V> / PointSet
  if (tsType.startsWith("PointMap<")) {
    const inner = tsType.match(/^PointMap<(.+)>$/);
    return inner ? `HashMap<Point, ${mapType(inner[1])}>` : "HashMap<Point, ()>";
  }
  if (tsType === "PointSet") return "HashSet<Point>";

  // Nullable T | null or T | undefined
  if (tsType.includes(" | null") || tsType.includes(" | undefined")) {
    const base = tsType.replace(/ \| null/g, "").replace(/ \| undefined/g, "").trim();
    return `Option<${mapType(base)}>`;
  }

  // Direct lookup
  if (TYPE_MAP[tsType]) return TYPE_MAP[tsType];

  // Unknown — keep as-is with a comment
  return `/* TS: ${tsType} */ todo_type`;
}

function mapParamType(tsType: string, paramName: string): { rustType: string; refPrefix: string } {
  const mapped = mapType(tsType);
  // Large types get &/&mut
  const byRef = [
    "VisibilityGraph",
    "ObstacleTree",
    "TransientGraphUtility",
    "Rectangle",
    "Polyline",
    "ScanSegmentTree",
  ];
  const mutable = ["VisibilityGraph", "ObstacleTree", "TransientGraphUtility", "ScanSegmentTree"];

  for (const t of byRef) {
    if (mapped === t || mapped.includes(t)) {
      const isMut = mutable.some((m) => mapped === m || mapped.includes(m));
      return { rustType: mapped, refPrefix: isMut ? "&mut " : "& " };
    }
  }
  return { rustType: mapped, refPrefix: "" };
}

// ── Branch structure extraction ──────────────────────────────────────────

interface BranchInfo {
  kind: string; // "if" | "else" | "for" | "while" | "switch"
  line: number;
  condition?: string;
}

function extractBranches(node: ts.Node, sourceFile: ts.SourceFile): BranchInfo[] {
  const branches: BranchInfo[] = [];

  function visit(n: ts.Node) {
    const line = sourceFile.getLineAndCharacterOfPosition(n.getStart(sourceFile)).line + 1;
    if (ts.isIfStatement(n)) {
      const cond = n.expression.getText(sourceFile).substring(0, 60);
      branches.push({ kind: "if", line, condition: cond });
    } else if (ts.isForStatement(n) || ts.isForInStatement(n) || ts.isForOfStatement(n)) {
      branches.push({ kind: "for", line });
    } else if (ts.isWhileStatement(n) || ts.isDoStatement(n)) {
      branches.push({ kind: "while", line });
    } else if (ts.isSwitchStatement(n)) {
      branches.push({ kind: "switch", line });
    }
    ts.forEachChild(n, visit);
  }
  visit(node);
  return branches;
}

// ── Method call extraction ───────────────────────────────────────────────

function extractCalls(node: ts.Node, sourceFile: ts.SourceFile): string[] {
  const calls = new Set<string>();
  function visit(n: ts.Node) {
    if (ts.isCallExpression(n)) {
      const expr = n.expression.getText(sourceFile);
      // Simplify: take last segment of property access
      const parts = expr.split(".");
      const name = parts[parts.length - 1];
      if (name && !name.startsWith("(") && name !== "push" && name !== "clear") {
        calls.add(expr.length > 50 ? name : expr);
      }
    }
    ts.forEachChild(n, visit);
  }
  visit(node);
  return Array.from(calls);
}

// ── Name conversion ──────────────────────────────────────────────────────

function toSnakeCase(name: string): string {
  return name
    .replace(/([A-Z])/g, "_$1")
    .toLowerCase()
    .replace(/^_/, "")
    .replace(/__/g, "_");
}

// ── Main generator ───────────────────────────────────────────────────────

interface MethodInfo {
  name: string;
  rustName: string;
  params: { name: string; tsType: string; rustType: string; refPrefix: string }[];
  returnType: string;
  rustReturnType: string;
  isStatic: boolean;
  isPrivate: boolean;
  startLine: number;
  endLine: number;
  lineCount: number;
  branches: BranchInfo[];
  calls: string[];
  bodyText: string;
}

function parseFile(filePath: string): {
  className: string | null;
  methods: MethodInfo[];
  fields: { name: string; tsType: string; rustType: string }[];
  sourceFile: ts.SourceFile;
} {
  let source = fs.readFileSync(filePath, "utf-8");

  // If the excerpt doesn't contain a class declaration, wrap it in a dummy class
  // so the TS parser can recognize the methods.
  let wrapped = false;
  if (!source.includes("class ")) {
    source = `class _Wrapper {\n${source}\n}`;
    wrapped = true;
  }

  const sourceFile = ts.createSourceFile(filePath, source, ts.ScriptTarget.Latest, true);

  let className: string | null = null;
  const methods: MethodInfo[] = [];
  const fields: { name: string; tsType: string; rustType: string }[] = [];

  function visit(node: ts.Node) {
    // Class declaration
    if (ts.isClassDeclaration(node) && node.name) {
      className = node.name.text;
    }

    // Method declarations
    if (ts.isMethodDeclaration(node) && node.name) {
      const name = node.name.getText(sourceFile);
      const isStatic = node.modifiers?.some((m) => m.kind === ts.SyntaxKind.StaticKeyword) ?? false;
      const isPrivate =
        node.modifiers?.some((m) => m.kind === ts.SyntaxKind.PrivateKeyword) ?? false;

      // Parameters
      const params = node.parameters.map((p) => {
        const paramName = p.name.getText(sourceFile);
        const tsType = p.type ? p.type.getText(sourceFile) : "any";
        const { rustType, refPrefix } = mapParamType(tsType, paramName);
        return { name: toSnakeCase(paramName), tsType, rustType, refPrefix };
      });

      // Return type
      const returnTypeNode = node.type;
      const tsReturnType = returnTypeNode ? returnTypeNode.getText(sourceFile) : "void";
      const rustReturnType = mapType(tsReturnType);

      // Line numbers
      const startLine =
        sourceFile.getLineAndCharacterOfPosition(node.getStart(sourceFile)).line + 1;
      const endLine = sourceFile.getLineAndCharacterOfPosition(node.getEnd()).line + 1;
      const lineCount = endLine - startLine + 1;

      // Branch structure & calls
      const branches = node.body ? extractBranches(node.body, sourceFile) : [];
      const calls = node.body ? extractCalls(node.body, sourceFile) : [];

      // Body text (for reference)
      const bodyText = node.body ? node.body.getText(sourceFile) : "";

      methods.push({
        name,
        rustName: toSnakeCase(name),
        params,
        returnType: tsReturnType,
        rustReturnType,
        isStatic,
        isPrivate,
        startLine,
        endLine,
        lineCount,
        branches,
        calls,
        bodyText,
      });
    }

    // Property declarations (fields)
    if (ts.isPropertyDeclaration(node) && node.name) {
      const name = node.name.getText(sourceFile);
      const tsType = node.type ? node.type.getText(sourceFile) : "any";
      fields.push({ name: toSnakeCase(name), tsType, rustType: mapType(tsType) });
    }

    ts.forEachChild(node, visit);
  }

  visit(sourceFile);
  // If we wrapped, adjust line numbers back (subtract 1 for the added class line)
  if (wrapped) {
    for (const m of methods) {
      m.startLine -= 1;
      m.endLine -= 1;
      for (const b of m.branches) {
        b.line -= 1;
      }
    }
  }

  return { className, methods, fields, sourceFile };
}

function generateRust(
  parsed: ReturnType<typeof parseFile>,
  structName: string,
  inputPath: string,
): string {
  const lines: string[] = [];
  const inputBasename = path.basename(inputPath);

  lines.push(`//! Auto-generated skeleton from \`${inputBasename}\`.`);
  lines.push(`//! Fill in the todo!() bodies by translating the TS reference line-by-line.`);
  lines.push(`//!`);
  lines.push(`//! Reference: \`reference/${inputBasename}\``);
  lines.push(``);
  lines.push(`use crate::geometry::point::Point;`);
  lines.push(`use crate::geometry::rectangle::Rectangle;`);
  lines.push(`use crate::geometry::point_comparer::GeomConstants;`);
  lines.push(`use crate::visibility::graph::{VertexId, VisibilityGraph};`);
  lines.push(`use super::compass_direction::CompassDirection;`);
  lines.push(`use super::obstacle_tree::ObstacleTree;`);
  lines.push(`use super::transient_graph_utility::TransientGraphUtility;`);
  lines.push(``);

  // If we have methods, emit impl block
  if (parsed.methods.length > 0) {
    lines.push(`impl ${structName} {`);

    for (const m of parsed.methods) {
      lines.push(``);
      // Comment block with TS reference info
      lines.push(`    // ── TS: ${inputBasename} lines ${m.startLine}-${m.endLine} (${m.lineCount} lines) ──`);
      lines.push(`    //`);

      // Branch structure
      if (m.branches.length > 0) {
        lines.push(`    // Branches:`);
        for (const b of m.branches) {
          if (b.condition) {
            lines.push(`    //   ${b.kind} (line ${b.line}): ${b.condition}`);
          } else {
            lines.push(`    //   ${b.kind} (line ${b.line})`);
          }
        }
      }

      // Key calls
      if (m.calls.length > 0) {
        lines.push(`    // Calls: ${m.calls.join(", ")}`);
      }

      // Visibility
      const vis = m.isPrivate ? "fn" : "pub fn";

      // Self parameter
      const selfParam = m.isStatic ? "" : "&mut self, ";

      // Parameters
      const params = m.params
        .map((p) => `${p.name}: ${p.refPrefix}${p.rustType}`)
        .join(", ");

      // Return type
      const ret = m.rustReturnType === "()" ? "" : ` -> ${m.rustReturnType}`;

      lines.push(`    ${vis} ${m.rustName}(${selfParam}${params})${ret} {`);
      lines.push(`        todo!()`);
      lines.push(`    }`);
    }

    lines.push(`}`);
  }

  lines.push(``);
  return lines.join("\n");
}

// ── CLI ──────────────────────────────────────────────────────────────────

function main() {
  const args = process.argv.slice(2);
  if (args.length < 2) {
    console.error("Usage: npx ts-node generate.ts <input.ts> <output.rs> [--struct Name]");
    process.exit(1);
  }

  const inputPath = args[0];
  const outputPath = args[1];
  let structName = "FullPortManager"; // default

  const structIdx = args.indexOf("--struct");
  if (structIdx !== -1 && args[structIdx + 1]) {
    structName = args[structIdx + 1];
  }

  if (!fs.existsSync(inputPath)) {
    console.error(`Input file not found: ${inputPath}`);
    process.exit(1);
  }

  const parsed = parseFile(inputPath);
  const rust = generateRust(parsed, structName, inputPath);

  if (outputPath === "/dev/stdout" || outputPath === "-") {
    process.stdout.write(rust);
  } else {
    fs.writeFileSync(outputPath, rust);
    console.log(`Generated ${outputPath} (${parsed.methods.length} methods)`);
  }
}

main();
