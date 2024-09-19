use crate::{body::Body, math_utils::Vec2};

#[derive(Debug, Clone, Copy)]
pub enum EdgeNumbers {
    NoEdge = 0,
    Edge1,
    Edge2,
    Edge3,
    Edge4,
}

#[derive(Debug, Clone, Copy)]
pub struct Edges {
    pub in_edge_1: EdgeNumbers,
    pub out_edge_1: EdgeNumbers,
    pub in_edge_2: EdgeNumbers,
    pub out_edge_2: EdgeNumbers,
}

impl Default for Edges {
    fn default() -> Self {
        Self {
            in_edge_1: EdgeNumbers::NoEdge,
            out_edge_1: EdgeNumbers::NoEdge,
            in_edge_2: EdgeNumbers::NoEdge,
            out_edge_2: EdgeNumbers::NoEdge,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FeaturePair {
    pub edges: Edges,
    pub value: i32,
}

impl FeaturePair {
    pub fn new(edges: Edges, value: i32) -> Self {
        Self { edges, value }
    }
}
#[derive(Debug)]
pub struct Contact {
    pub position: Vec2,
    pub normal: Vec2,
    pub r1: Vec2,
    pub r2: Vec2,
    pub separation: f32,
    pub Pn: f32,  // sum of impulse in the direction of normal
    pub Pt: f32,  // sum of impulse in the direction of tangent
    pub Pnb: f32, // sum of impulse for position bias
    pub mass_normal: f32,
    pub mass_tangent: f32,
    pub bias: f32,
    pub feature: FeaturePair,
}

impl Default for Contact {
    fn default() -> Self {
        Self {
            position: Vec2::new(0.0, 0.0),
            normal: Vec2::new(0.0, 0.0),
            r1: Vec2::new(0.0, 0.0),
            r2: Vec2::new(0.0, 0.0),
            separation: Default::default(),
            Pn: Default::default(),
            Pt: Default::default(),
            Pnb: Default::default(),
            mass_normal: Default::default(),
            mass_tangent: Default::default(),
            bias: Default::default(),
            feature: FeaturePair::new(Edges::default(), 0),
        }
    }
}

struct ArbiterKey {
    body1: Body,
    body2: Body,
}

struct Arbiter {
    body1: Body,
    body2: Body,
    friction: f32,
    num_contacts: i32,
    contacts: [Contact; 2],
}
