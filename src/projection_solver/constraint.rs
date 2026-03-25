use super::variable::VarIndex;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ConIndex(pub usize);

#[derive(Clone, Debug)]
pub struct Constraint {
    pub index: ConIndex,
    pub left: VarIndex,
    pub right: VarIndex,
    pub gap: f64,
}
