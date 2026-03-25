#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VarIndex(pub usize);

#[derive(Clone, Debug)]
pub struct Variable {
    pub index: VarIndex,
    pub desired_pos: f64,
    pub actual_pos: f64,
    pub weight: f64,
    pub scale: f64,
}
