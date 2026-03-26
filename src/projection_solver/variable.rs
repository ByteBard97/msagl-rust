use super::block::BlockIndex;
use super::constraint::ConIndex;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VarIndex(pub usize);

#[derive(Clone, Debug)]
pub struct NeighborAndWeight {
    pub neighbor: VarIndex,
    pub weight: f64,
}

#[derive(Clone, Debug)]
pub struct Variable {
    pub index: VarIndex,
    pub desired_pos: f64,
    pub actual_pos: f64,
    pub weight: f64,
    pub scale: f64,
    pub offset_in_block: f64,
    pub block: BlockIndex,
    pub active_constraint_count: u32,
    pub left_constraints: Vec<ConIndex>,
    pub right_constraints: Vec<ConIndex>,
    pub neighbors: Vec<NeighborAndWeight>,
    pub ordinal: u32,
}

impl Variable {
    pub fn new(index: VarIndex, desired_pos: f64, weight: f64, scale: f64) -> Self {
        Self {
            index,
            desired_pos,
            actual_pos: desired_pos,
            weight,
            scale,
            offset_in_block: 0.0,
            block: BlockIndex(0),
            active_constraint_count: 0,
            left_constraints: Vec::new(),
            right_constraints: Vec::new(),
            neighbors: Vec::new(),
            ordinal: 0,
        }
    }

    #[inline]
    pub fn dfdv(&self) -> f64 {
        (2.0 * self.weight * (self.actual_pos - self.desired_pos)) / self.scale
    }
}
