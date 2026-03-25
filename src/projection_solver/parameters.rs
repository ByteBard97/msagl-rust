#[derive(Clone, Debug)]
pub struct Parameters {
    pub gap_tolerance: f64,
    pub qpsc_convergence_epsilon: f64,
    pub qpsc_convergence_quotient: f64,
    pub outer_project_iterations_limit: i32,
    pub inner_project_iterations_limit: i32,
    pub time_limit_ms: i32,
    pub advanced: AdvancedParameters,
}

#[derive(Clone, Debug)]
pub struct AdvancedParameters {
    pub force_qpsc: bool,
    pub scale_in_qpsc: bool,
    pub min_split_lagrangian_threshold: f64,
    pub use_violation_cache: bool,
    pub violation_cache_min_blocks_divisor: usize,
    pub violation_cache_min_blocks_count: usize,
}

impl Default for Parameters {
    fn default() -> Self {
        Self {
            gap_tolerance: 1e-4,
            qpsc_convergence_epsilon: 1e-5,
            qpsc_convergence_quotient: 1e-6,
            outer_project_iterations_limit: -1,
            inner_project_iterations_limit: -1,
            time_limit_ms: -1,
            advanced: AdvancedParameters::default(),
        }
    }
}

impl Default for AdvancedParameters {
    fn default() -> Self {
        Self {
            force_qpsc: false,
            scale_in_qpsc: true,
            min_split_lagrangian_threshold: -1e-7,
            use_violation_cache: true,
            violation_cache_min_blocks_divisor: 10,
            violation_cache_min_blocks_count: 100,
        }
    }
}
