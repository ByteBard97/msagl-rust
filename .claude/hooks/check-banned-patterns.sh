#!/bin/bash
# check-banned-patterns.sh
# PreToolUse hook for Write and Edit tools.
# Blocks any write to src/ or tests/ that contains banned patterns.
# Claude sees the reason and must fix the issue before proceeding.

INPUT=$(cat)

TOOL=$(echo "$INPUT" | jq -r '.tool_name // empty')
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

# Only check files inside src/ or tests/
if [[ "$FILE_PATH" != */src/* ]] && [[ "$FILE_PATH" != */tests/* ]]; then
    exit 0
fi

# Get content being written (Write tool uses "content", Edit uses "new_string")
CONTENT=$(echo "$INPUT" | jq -r '.tool_input.content // .tool_input.new_string // empty')

deny() {
    local reason="$1"
    jq -n --arg reason "$reason" '{
        "hookSpecificOutput": {
            "hookEventName": "PreToolUse",
            "permissionDecision": "deny",
            "permissionDecisionReason": $reason
        }
    }'
    exit 0
}

# ── Banned in src/ ──────────────────────────────────────────────────────────

if [[ "$FILE_PATH" == */src/* ]]; then
    if echo "$CONTENT" | grep -q 'todo!()'; then
        deny "BLOCKED: todo!() is not allowed in src/. Port the real algorithm from C#/TS instead."
    fi
    if echo "$CONTENT" | grep -q 'unimplemented!()'; then
        deny "BLOCKED: unimplemented!() is not allowed in src/. Port the real algorithm from C#/TS instead."
    fi
    if echo "$CONTENT" | grep -qE '// TODO|// FIXME'; then
        deny "BLOCKED: // TODO and // FIXME comments are not allowed in src/. Fix the issue now or port the missing feature."
    fi
    if echo "$CONTENT" | grep -q '// SIMPLIFIED'; then
        deny "BLOCKED: // SIMPLIFIED comment found. This codebase requires faithful ports, not simplifications."
    fi
fi

# ── Banned in src/routing/ ───────────────────────────────────────────────────

if [[ "$FILE_PATH" == */src/routing/* ]]; then
    if echo "$CONTENT" | grep -qi 'bbox approximat'; then
        deny "BLOCKED: bbox approximation found in routing code. Non-rectangular obstacles must use actual polygon routing, not bounding box shortcuts."
    fi
    if echo "$CONTENT" | grep -q 'add_nonrect_placeholder'; then
        deny "BLOCKED: add_nonrect_placeholder is a banned bbox shortcut. Use b.add_polygon() with actual polygon points."
    fi
    if echo "$CONTENT" | grep -qi 'bounding.box.*non.rect\|non.rect.*bounding.box\|placeholder for non'; then
        deny "BLOCKED: bbox placeholder for non-rectangular obstacle found. Port the real ICurve polygon routing from C#/TS."
    fi
fi

# ── Banned in tests/ ────────────────────────────────────────────────────────

if [[ "$FILE_PATH" == */tests/* ]]; then
    if echo "$CONTENT" | grep -q '#\[ignore'; then
        deny "BLOCKED: #[ignore] is not allowed in tests/. Fix the underlying implementation instead of hiding the failure."
    fi
    if echo "$CONTENT" | grep -q 'add_nonrect_placeholder'; then
        deny "BLOCKED: add_nonrect_placeholder is banned in tests/. Tests must use b.add_polygon() with real polygon geometry. Tests are expected to fail until the implementation is fixed — that is correct and honest."
    fi
fi

exit 0
