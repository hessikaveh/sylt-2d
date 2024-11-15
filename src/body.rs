use crate::math_utils::Vec2;
use std::sync::atomic::{AtomicUsize, Ordering};

#[derive(Debug, Default, Copy, Clone)]
pub struct Body {
    pub id: usize,
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

static BODY_ID_COUNTER: AtomicUsize = AtomicUsize::new(1);

impl Body {
    pub fn new(width: Vec2, mass: f32) -> Self {
        let inv_mass;
        let inv_moi;
        let moi;
        if mass < f32::MAX {
            inv_mass = 1.0 / mass;
            moi = mass * (width.x * width.x + width.y * width.y) / 12.0;
            inv_moi = 1.0 / moi;
        } else {
            inv_mass = 0.0;
            moi = f32::MAX;
            inv_moi = 0.0;
        }

        let id = BODY_ID_COUNTER.fetch_add(1, Ordering::Relaxed);

        Self {
            id,
            position: Vec2::new(0.0, 0.0),
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            friction: 0.0,
            width,
            mass,
            inv_mass,
            inv_moi,
            moi,
        }
    }

    pub fn add_force(&mut self, force: Vec2) {
        self.force = self.force + force;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_body() {
        let body = Body::default();
        println!("{:?}", body);

        let body = Body::new(Vec2::new(1.0, 5.0), 50.0);
        println!("{:?}", body);
    }
    #[test]
    fn test_add_force() {
        let mut body = Body::default();
        body.add_force(Vec2::new(2.0, 5.3));
        assert_eq!(body.force, Vec2::new(2.0, 5.3));
    }
}
