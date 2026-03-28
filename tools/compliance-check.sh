#!/bin/bash
# compliance-check.sh
# Full compliance gate for msagl-rust faithful port.
# Exits non-zero and prints every violation found.
# Run: bash tools/compliance-check.sh
# Used as: git pre-commit hook + Claude Code PostToolUse hook

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAIL=0

red()   { printf '\033[31m%s\033[0m\n' "$*"; }
green() { printf '\033[32m%s\033[0m\n' "$*"; }
warn()  { printf '\033[33m%s\033[0m\n' "$*"; }
info()  { printf '%s\n' "$*"; }

section() { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }

cd "$REPO"

# ─────────────────────────────────────────────────────────────────────────────
section "1. Banned patterns in src/"
# ─────────────────────────────────────────────────────────────────────────────

check_banned() {
    local label="$1"
    local pattern="$2"
    local path="$3"
    local results
    results=$(grep -rn "$pattern" "$path" 2>/dev/null || true)
    if [ -n "$results" ]; then
        red "FAIL [$label]"
        echo "$results" | sed 's/^/  /'
        FAIL=1
    else
        green "PASS [$label]"
    fi
}

check_banned "no todo!()"                '^[^/]*todo!()'                 src/
check_banned "no unimplemented!()"       'unimplemented!()'              src/
check_banned "no // TODO in routing"     '// TODO'                       src/routing/
check_banned "no // FIXME in routing"    '// FIXME'                      src/routing/
check_banned "no // SIMPLIFIED"          '// SIMPLIFIED'                 src/
check_banned "no bbox approximat"        'bbox approximat'               src/routing/
check_banned "no add_nonrect_placeholder" 'add_nonrect_placeholder'      src/
check_banned "no placeholder comments"  'placeholder for non-rect'      src/

# ─────────────────────────────────────────────────────────────────────────────
section "2. No #[ignore] in tests/"
# ─────────────────────────────────────────────────────────────────────────────

# Match only actual #[ignore] attributes, not doc comments mentioning them
ignored=$(grep -rn '^#\[ignore' tests/ 2>/dev/null || true)
if [ -n "$ignored" ]; then
    red "FAIL [no #[ignore] in tests/]"
    echo "$ignored" | sed 's/^/  /'
    FAIL=1
else
    green "PASS [no #[ignore] in tests/]"
fi

# ─────────────────────────────────────────────────────────────────────────────
section "3. C# test parity (all 243 tests must be ported)"
# ─────────────────────────────────────────────────────────────────────────────

CS_FILE="../MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs"
if [ ! -f "$CS_FILE" ]; then
    warn "SKIP [C# test parity — reference file not found at $CS_FILE]"
else
    # Normalise: lowercase, strip underscores
    grep "public void " "$CS_FILE" \
        | sed 's/.*public void //' \
        | sed 's/(.*//' \
        | tr '[:upper:]' '[:lower:]' \
        | tr -d '_' \
        | sort > "${TMPDIR:-/tmp}/msagl_cs_tests.txt"

    grep -rh "^fn " tests/*.rs \
        | sed 's/fn //' \
        | sed 's/(.*//' \
        | tr '[:upper:]' '[:lower:]' \
        | tr -d '_' \
        | sort > "${TMPDIR:-/tmp}/msagl_rust_tests.txt"

    missing=$(comm -23 "${TMPDIR:-/tmp}/msagl_cs_tests.txt" "${TMPDIR:-/tmp}/msagl_rust_tests.txt")
    count=$(echo "$missing" | grep -c '[a-z]' || true)

    if [ "$count" -gt 0 ]; then
        red "FAIL [C# test parity — $count tests not ported]"
        echo "$missing" | sed 's/^/  /'
        FAIL=1
    else
        green "PASS [C# test parity — all 243 tests ported]"
    fi
fi

# ─────────────────────────────────────────────────────────────────────────────
section "4. cargo check (zero errors)"
# ─────────────────────────────────────────────────────────────────────────────

if cargo check 2>&1 | grep -q '^error'; then
    red "FAIL [cargo check has errors]"
    cargo check 2>&1 | grep '^error' | head -20 | sed 's/^/  /'
    FAIL=1
else
    green "PASS [cargo check]"
fi

# ─────────────────────────────────────────────────────────────────────────────
section "5. cargo test (zero failures, zero ignored)"
# ─────────────────────────────────────────────────────────────────────────────

test_output=$(cargo test -- --include-ignored 2>&1)
failures=$(echo "$test_output" | grep -E '^test result:.*FAILED|^FAILED' || true)
ignored_count=$(echo "$test_output" | grep 'test result:' | grep -oE '[0-9]+ ignored' | awk '{s+=$1} END {print s}')

if [ -n "$failures" ]; then
    red "FAIL [cargo test has failures]"
    echo "$test_output" | grep -E 'FAILED|^error' | head -20 | sed 's/^/  /'
    FAIL=1
else
    green "PASS [cargo test — no failures]"
fi

if [ "${ignored_count:-0}" -gt 0 ]; then
    red "FAIL [cargo test has $ignored_count ignored tests]"
    echo "$test_output" | grep 'ignored' | sed 's/^/  /'
    FAIL=1
else
    green "PASS [cargo test — zero ignored]"
fi

# ─────────────────────────────────────────────────────────────────────────────
section "6. api-diff (zero items missing from src/)"
# ─────────────────────────────────────────────────────────────────────────────

if ! command -v node &>/dev/null; then
    warn "SKIP [api-diff — node not found]"
elif [ ! -f "tools/api-diff.mjs" ]; then
    warn "SKIP [api-diff — tools/api-diff.mjs not found]"
else
    diff_output=$(node tools/api-diff.mjs --ts-only --no-ToString 2>&1)
    missing_count=$(echo "$diff_output" | grep 'Missing entirely from src/:' | grep -oE '[0-9]+' || echo 0)
    if [ "${missing_count:-0}" -gt 0 ]; then
        red "FAIL [api-diff — $missing_count items missing from src/]"
        echo "$diff_output" | grep 'MISSING FROM src/ ENTIRELY' -A 3 | sed 's/^/  /'
        FAIL=1
    else
        green "PASS [api-diff — zero items missing from src/]"
    fi
fi

# ─────────────────────────────────────────────────────────────────────────────
section "Summary"
# ─────────────────────────────────────────────────────────────────────────────

if [ "$FAIL" -eq 0 ]; then
    green "ALL CHECKS PASSED"
    exit 0
else
    red "COMPLIANCE FAILURES DETECTED — fix all issues above before committing"
    exit 1
fi
