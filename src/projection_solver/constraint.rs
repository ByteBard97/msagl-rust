use super::variable::VarIndex;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ConIndex(pub usize);

#[derive(Clone, Debug)]
pub struct Constraint {
    pub index: ConIndex,
    pub left: VarIndex,
    pub right: VarIndex,
    pub gap: f64,
    pub is_equality: bool,
    pub is_active: bool,
    pub is_unsatisfiable: bool,
    pub lagrangian: f64,
    pub vector_index: usize,
}

impl Constraint {
    pub fn new(index: ConIndex, left: VarIndex, right: VarIndex, gap: f64, is_equality: bool) -> Self {
        Self {
            index,
            left,
            right,
            gap,
            is_equality,
            is_active: false,
            is_unsatisfiable: false,
            lagrangian: 0.0,
            vector_index: 0,
        }
    }

    #[inline]
    pub fn violation(&self, left_actual: f64, left_scale: f64, right_actual: f64, right_scale: f64) -> f64 {
        left_actual * left_scale + self.gap - right_actual * right_scale
    }
}
