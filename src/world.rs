use std::collections::HashMap;

use crate::arbiter::{Arbiter, ArbiterKey};
use crate::body::Body;
use crate::joint::Joint;
use crate::math_utils::Vec2;

struct World {
    gravity: Vec2,
    iterations: i32,
    accumulate_impulse: bool,
    warm_starting: bool,
    position_correction: bool,
    bodies: Vec<Body>,
    joints: Vec<Joint>,
    arbiters: HashMap<ArbiterKey, Arbiter>,
}
