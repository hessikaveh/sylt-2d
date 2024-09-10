use crate::{body::Body, math_utils::Vec2};

enum Edges {
    inEdge1(String),
    outEdge1(String),
    inEdge2(String),
    outEdge2(String),
}

struct FeaturePair {
    edges: Edges,
    value: i32,
}

pub struct Contact {
    position: Vec2,
    normal: Vec2,
    r1: Vec2,
    r2: Vec2,
    separation: f32,
    Pn: f32,  // sum of impulse in the direction of normal
    Pt: f32,  // sum of impulse in the direction of tangent
    Pnb: f32, // sum of impulse for position bias
    mass_normal: f32,
    mass_tangent: f32,
    bias: f32,
    feature: FeaturePair,
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
