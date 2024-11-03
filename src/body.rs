use crate::math_utils::Vec2;
use std::hash::{Hash, Hasher};

#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub struct Body {
    pub position: Vec2,
    pub rotation: f32,
    pub velocity: Vec2,
    pub angular_velocity: f32,
    pub force: Vec2,
    pub torque: f32,
    pub width: Vec2,
    pub friction: f32,
    pub mass: f32,
    pub inv_mass: f32,
    pub moi: f32,
    pub inv_moi: f32,
}

impl Hash for Body {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.position.hash(state); // Hash the Vec2 by converting it to an array
        self.rotation.to_bits().hash(state);
        self.velocity.hash(state);
        self.angular_velocity.to_bits().hash(state);
        self.force.hash(state);
        self.torque.to_bits().hash(state);
        self.width.hash(state);
        self.friction.to_bits().hash(state);
        self.mass.to_bits().hash(state);
        self.inv_mass.to_bits().hash(state);
        self.moi.to_bits().hash(state);
        self.inv_moi.to_bits().hash(state);
    }
}

impl Eq for Body {}

impl Default for Body {
    fn default() -> Self {
        Self::new()
    }
}

impl Body {
    pub fn new() -> Self {
        Self {
            position: Vec2::new(0.0, 0.0),
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            friction: 0.0,
            width: Vec2::new(0.0, 0.0),
            mass: 0.0,
            inv_mass: 0.0,
            inv_moi: 0.0,
            moi: 0.0,
        }
    }

    pub fn add_force(&mut self, force: Vec2) {
        self.force = self.force + force;
    }

    pub fn set_weight_mass(&mut self, mass: f32) {
        self.mass = mass;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_body() {
        let body = Body::new();
        println!("{:?}", body);
    }
    #[test]
    fn test_add_force() {
        let mut body = Body::new();
        body.add_force(Vec2::new(2.0, 5.3));
        assert_eq!(body.force, Vec2::new(2.0, 5.3));
    }
}
