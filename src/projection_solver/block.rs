use super::variable::VarIndex;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BlockIndex(pub usize);

#[derive(Clone, Debug)]
pub struct Block {
    pub index: BlockIndex,
    pub variables: Vec<VarIndex>,
    pub reference_pos: f64,
    pub scale: f64,
}
