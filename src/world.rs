use std::collections::HashMap;

use crate::arbiter::{Arbiter, ArbiterKey};
use crate::body::Body;
use crate::joint::Joint;
use crate::math_utils::Vec2;

#[derive(Clone, Copy)]
pub struct WorldContext {
    pub accumulate_impulse: bool,
    pub warm_starting: bool,
    pub position_correction: bool,
}
pub struct World {
    gravity: Vec2,
    iterations: i32,
    pub world_context: WorldContext,
    pub bodies: Vec<Body>,
    pub joints: Vec<Joint>,
    pub arbiters: HashMap<ArbiterKey, Arbiter>,
}

impl World {
    pub fn new(gravity: Vec2, iterations: i32) -> Self {
        let context = WorldContext {
            accumulate_impulse: true,
            warm_starting: true,
            position_correction: true,
        };
        Self {
            gravity,
            iterations,
            world_context: context,
            bodies: vec![],
            joints: vec![],
            arbiters: HashMap::<ArbiterKey, Arbiter>::new(),
        }
    }

    pub fn add_body(&mut self, body: Body) {
        self.bodies.push(body);
    }

    pub fn add_joint(&mut self, joint: Joint) {
        self.joints.push(joint);
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.joints.clear();
        self.arbiters.clear();
    }

    pub fn broad_phase(&mut self) {
        for i in 0..self.bodies.len() {
            let body_i = self.bodies[i];

            for j in (i + 1)..self.bodies.len() {
                let body_j = self.bodies[j];
                if body_i.inv_mass == 0.0 && body_j.inv_mass == 0.0 {
                    continue;
                };
                let new_arbiter = Arbiter::new(self.bodies[i], self.bodies[j]);
                let key = ArbiterKey::new(&body_i, &body_j);

                if new_arbiter.num_contacts > 0 {
                    let _ = self
                        .arbiters
                        .entry(key)
                        .and_modify(|arbiter| {
                            arbiter.update(
                                new_arbiter.contacts.as_ref(),
                                new_arbiter.num_contacts,
                                &self.world_context,
                            )
                        })
                        .or_insert(new_arbiter);
                } else {
                    self.arbiters.remove(&key);
                }
            }
        }
    }

    pub fn step(&mut self, dt: f32) {
        let inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };
        // Determine overlapping bodies and update contact points.
        self.broad_phase();

        // Integrate forces.
        for body in self.bodies.iter_mut() {
            if body.inv_mass == 0.0 {
                continue;
            };
            body.velocity = body.velocity + (self.gravity + body.force * body.inv_mass) * dt;
            body.angular_velocity += body.inv_moi * body.torque * dt;
        }

        // Pefrom pre-steps
        for (_, arbiter) in self.arbiters.iter_mut() {
            arbiter.pre_step(inv_dt, &self.world_context);
        }

        for joint in self.joints.iter_mut() {
            joint.pre_step(&self.world_context, inv_dt);
        }

        // Perfrom iterations
        for _ in 0..self.iterations {
            for (_, arbiter) in self.arbiters.iter_mut() {
                arbiter.apply_impulse(&self.world_context);
            }

            for joint in self.joints.iter_mut() {
                joint.apply_impulse();
            }
        }

        // Integrate Velocities
        for body in self.bodies.iter_mut() {
            body.position = body.position + body.velocity * dt;
            body.rotation += body.angular_velocity * dt;

            body.force = Vec2::default();
            body.torque = 0.0;
        }
    }
}
