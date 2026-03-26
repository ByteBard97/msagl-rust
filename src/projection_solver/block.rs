use super::variable::VarIndex;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BlockIndex(pub usize);

#[derive(Clone, Debug)]
pub struct Block {
    pub index: BlockIndex,
    pub variables: Vec<VarIndex>,
    pub reference_pos: f64,
    pub scale: f64,
    pub sum_a2: f64,
    pub sum_ad: f64,
    pub sum_ab: f64,
    pub vector_index: usize,
}

impl Block {
    pub fn new(
        index: BlockIndex,
        var: VarIndex,
        desired_pos: f64,
        weight: f64,
        scale: f64,
    ) -> Self {
        Self {
            index,
            variables: vec![var],
            reference_pos: desired_pos,
            scale,
            sum_a2: weight, // a=1 for first var, a²*w = w
            sum_ad: desired_pos * weight,
            sum_ab: 0.0,
            vector_index: 0,
        }
    }

    pub fn add_variable_with_offset(
        &mut self,
        var: VarIndex,
        offset: f64,
        weight: f64,
        var_scale: f64,
        desired_pos: f64,
    ) {
        let a = self.scale / var_scale;
        let b = offset / var_scale;
        self.sum_a2 += a * a * weight;
        self.sum_ad += a * desired_pos * weight;
        self.sum_ab += a * b * weight;
        self.variables.push(var);
    }

    pub fn update_reference_pos(&mut self) {
        if self.sum_a2 > 0.0 {
            self.reference_pos = (self.sum_ad - self.sum_ab) / self.sum_a2;
        }
    }

    #[inline]
    pub fn compute_variable_pos(&self, offset: f64, var_scale: f64) -> f64 {
        (self.scale * self.reference_pos + offset) / var_scale
    }

    pub fn reset_sums(&mut self) {
        self.sum_a2 = 0.0;
        self.sum_ad = 0.0;
        self.sum_ab = 0.0;
    }

    pub fn add_to_sums(&mut self, offset: f64, weight: f64, var_scale: f64, desired_pos: f64) {
        let a = self.scale / var_scale;
        let b = offset / var_scale;
        self.sum_a2 += a * a * weight;
        self.sum_ad += a * desired_pos * weight;
        self.sum_ab += a * b * weight;
    }
}
