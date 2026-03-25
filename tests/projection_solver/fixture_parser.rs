/// Parser for C# projection solver golden-baseline fixture files.
///
/// Format overview (see task description for full details):
///   - Comment lines start with `//`
///   - Header lines: `Seed`, `Weight`, `Goal`, `UnsatisfiableConstraints`
///   - Sections delimited by `BEGIN X` / `END X` where X is
///     VARIABLES, CONSTRAINTS, NEIGHBOURS, RESULTS

#[derive(Debug, Clone)]
pub struct FixtureVariable {
    pub id: usize,
    pub desired_pos: f64,
    pub weight: f64,
    pub scale: f64,
}

#[derive(Debug, Clone)]
pub struct FixtureConstraint {
    pub left: usize,
    pub right: usize,
    pub gap: f64,
    pub is_equality: bool,
}

#[derive(Debug, Clone)]
pub struct FixtureNeighbor {
    pub var1: usize,
    pub var2: usize,
    pub weight: f64,
}

#[derive(Debug, Clone)]
pub struct FixtureResult {
    pub id: usize,
    pub expected_pos: f64,
}

#[derive(Debug, Clone)]
pub struct Fixture {
    pub variables: Vec<FixtureVariable>,
    pub constraints: Vec<FixtureConstraint>,
    pub neighbors: Vec<FixtureNeighbor>,
    pub results: Vec<FixtureResult>,
    pub expected_goal: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Section {
    Header,
    Variables,
    Constraints,
    Neighbours,
    Results,
}

/// Parse a fixture file's content into a `Fixture` struct.
///
/// Lines starting with `//` are comments and ignored.
/// The `size` column (3rd) in VARIABLES is always 0.0 and skipped.
/// Scale column (5th) in VARIABLES is optional, defaults to 1.0.
/// Equality constraints have `=` prefix on the gap value.
pub fn parse_fixture(content: &str) -> Fixture {
    let mut fixture = Fixture {
        variables: Vec::new(),
        constraints: Vec::new(),
        neighbors: Vec::new(),
        results: Vec::new(),
        expected_goal: None,
    };

    let mut section = Section::Header;

    for raw_line in content.lines() {
        let line = raw_line.trim();

        // Skip empty lines and comments
        if line.is_empty() || line.starts_with("//") {
            continue;
        }

        // Section transitions
        if line.starts_with("BEGIN ") {
            section = match &line[6..] {
                "VARIABLES" => Section::Variables,
                "CONSTRAINTS" => Section::Constraints,
                "NEIGHBOURS" => Section::Neighbours,
                "RESULTS" => Section::Results,
                other => panic!("Unknown section: {other}"),
            };
            continue;
        }
        if line.starts_with("END ") {
            section = Section::Header;
            continue;
        }

        match section {
            Section::Header => parse_header_line(line, &mut fixture),
            Section::Variables => fixture.variables.push(parse_variable(line)),
            Section::Constraints => fixture.constraints.push(parse_constraint(line)),
            Section::Neighbours => fixture.neighbors.push(parse_neighbor(line)),
            Section::Results => fixture.results.push(parse_result(line)),
        }
    }

    fixture
}

fn parse_header_line(line: &str, fixture: &mut Fixture) {
    if let Some(rest) = line.strip_prefix("Goal ") {
        fixture.expected_goal = Some(parse_f64(rest.trim()));
    }
    // Seed, Weight, UnsatisfiableConstraints — informational only
}

fn parse_variable(line: &str) -> FixtureVariable {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    // 4 columns: id desired_pos size weight  (scale defaults to 1.0)
    // 5 columns: id desired_pos size weight scale
    assert!(
        tokens.len() >= 4,
        "VARIABLES line needs at least 4 columns: {line}"
    );

    let id: usize = tokens[0].parse().expect("bad variable id");
    let desired_pos = parse_f64(tokens[1]);
    // tokens[2] is size — always 0.0, skip
    let weight = parse_f64(tokens[3]);
    let scale = if tokens.len() >= 5 {
        parse_f64(tokens[4])
    } else {
        1.0
    };

    FixtureVariable {
        id,
        desired_pos,
        weight,
        scale,
    }
}

fn parse_constraint(line: &str) -> FixtureConstraint {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    assert!(
        tokens.len() >= 3,
        "CONSTRAINTS line needs at least 3 columns: {line}"
    );

    let left: usize = tokens[0].parse().expect("bad constraint left");
    let right: usize = tokens[1].parse().expect("bad constraint right");
    let gap_str = tokens[2];

    let (gap, is_equality) = if let Some(stripped) = gap_str.strip_prefix('=') {
        (parse_f64(stripped), true)
    } else {
        (parse_f64(gap_str), false)
    };

    FixtureConstraint {
        left,
        right,
        gap,
        is_equality,
    }
}

fn parse_neighbor(line: &str) -> FixtureNeighbor {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    assert!(
        tokens.len() >= 3,
        "NEIGHBOURS line needs at least 3 columns: {line}"
    );

    FixtureNeighbor {
        var1: tokens[0].parse().expect("bad neighbor var1"),
        var2: tokens[1].parse().expect("bad neighbor var2"),
        weight: parse_f64(tokens[2]),
    }
}

fn parse_result(line: &str) -> FixtureResult {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    assert!(
        tokens.len() >= 2,
        "RESULTS line needs at least 2 columns: {line}"
    );

    FixtureResult {
        id: tokens[0].parse().expect("bad result id"),
        expected_pos: parse_f64(tokens[1]),
    }
}

/// Parse a floating-point number, handling scientific notation like `1.23E+16`.
fn parse_f64(s: &str) -> f64 {
    s.parse::<f64>()
        .unwrap_or_else(|e| panic!("Failed to parse f64 from '{s}': {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_simple_fixture() {
        let content = r#"
// @@Variables: 3
// @@Constraints: 2
// @@Neighbours: 1

Seed 0x0
Weight 1.00000
Goal 42.5

BEGIN VARIABLES
0 1.0 0.0 1.0
1 5.0 0.0 2.0
2 10.0 0.0 3.0 0.5
END VARIABLES

BEGIN CONSTRAINTS
0 1 3.00000
1 2 =5.00000
END CONSTRAINTS

BEGIN NEIGHBOURS
0 2 10.0
END NEIGHBOURS

BEGIN RESULTS
0 1.0
1 5.0
2 10.0
END RESULTS
"#;
        let f = parse_fixture(content);

        assert_eq!(f.variables.len(), 3);
        assert_eq!(f.variables[0].id, 0);
        assert_eq!(f.variables[0].desired_pos, 1.0);
        assert_eq!(f.variables[0].weight, 1.0);
        assert_eq!(f.variables[0].scale, 1.0);
        assert_eq!(f.variables[2].scale, 0.5);

        assert_eq!(f.constraints.len(), 2);
        assert!(!f.constraints[0].is_equality);
        assert_eq!(f.constraints[0].gap, 3.0);
        assert!(f.constraints[1].is_equality);
        assert_eq!(f.constraints[1].gap, 5.0);

        assert_eq!(f.neighbors.len(), 1);
        assert_eq!(f.neighbors[0].weight, 10.0);

        assert_eq!(f.results.len(), 3);
        assert_eq!(f.expected_goal, Some(42.5));
    }

    #[test]
    fn parse_scientific_notation_goal() {
        let content = r#"
Goal -6.52751886014641E+16

BEGIN VARIABLES
0 1.0 0.0 1.0
END VARIABLES

BEGIN CONSTRAINTS
END CONSTRAINTS

BEGIN RESULTS
0 1.0
END RESULTS
"#;
        let f = parse_fixture(content);
        assert!(f.expected_goal.unwrap() < -6e16);
    }

    #[test]
    fn parse_no_neighbours_section() {
        let content = r#"
Goal 100.0

BEGIN VARIABLES
0 5.0 0.0 1.0
END VARIABLES

BEGIN CONSTRAINTS
0 0 2.0
END CONSTRAINTS

BEGIN RESULTS
0 5.0
END RESULTS
"#;
        let f = parse_fixture(content);
        assert!(f.neighbors.is_empty());
    }
}
