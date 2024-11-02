use std::collections::HashMap;

use crate::arbiter::{Arbiter, ArbiterKey};
use crate::body::Body;
use crate::joint::Joint;
use crate::math_utils::Vec2;

pub struct World {
    gravity: Vec2,
    iterations: i32,
    pub accumulate_impulse: bool,
    pub warm_starting: bool,
    pub position_correction: bool,
    bodies: Vec<Body>,
    joints: Vec<Joint>,
    arbiters: HashMap<ArbiterKey, Arbiter>,
}

impl World {
    pub fn add_body(mut self, body: Body) {
        self.bodies.push(body);
    }

    pub fn add_join(mut self, joint: Joint) {
        self.joints.push(joint);
    }

    pub fn clear(mut self) {
        self.bodies.clear();
        self.joints.clear();
        self.arbiters.clear();
    }
}
