use crate::math_utils::Cross;
use crate::world::WorldContext;
use crate::{body::Body, collide::collide, math_utils::Vec2};

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
#[derive(Debug, Clone, Copy)]
pub struct Contact {
    pub position: Vec2,
    pub normal: Vec2,
    pub r1: Vec2,
    pub r2: Vec2,
    pub separation: f32,
    pub pn: f32,  // sum of impulse in the direction of normal
    pub pt: f32,  // sum of impulse in the direction of tangent
    pub pnb: f32, // sum of impulse for position bias
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
            pn: Default::default(),
            pt: Default::default(),
            pnb: Default::default(),
            mass_normal: Default::default(),
            mass_tangent: Default::default(),
            bias: Default::default(),
            feature: FeaturePair::new(Edges::default(), 0),
        }
    }
}

#[derive(Eq, Hash, PartialEq)]
pub struct ArbiterKey {
    body1: Body,
    body2: Body,
}

impl ArbiterKey {
    pub fn new(body_1: Body, body_2: Body) -> Self {
        if body_1 < body_2 {
            Self {
                body1: body_1,
                body2: body_2,
            }
        } else {
            Self {
                body1: body_2,
                body2: body_1,
            }
        }
    }
}

pub struct Arbiter {
    body1: Body,
    body2: Body,
    friction: f32,
    pub num_contacts: i32,
    pub contacts: [Contact; 2],
}

impl Arbiter {
    pub fn new(body_1: Body, body_2: Body) -> Self {
        let mut contacts = Vec::<Contact>::with_capacity(2);
        let num_contacts = collide(&mut contacts, &body_1, &body_2);
        if body_1 > body_2 {
            Self {
                body1: body_1,
                body2: body_2,
                friction: f32::sqrt(body_1.friction * body_2.friction),
                num_contacts,
                contacts: [contacts[0], contacts[1]],
            }
        } else {
            Self {
                body1: body_2,
                body2: body_1,
                friction: f32::sqrt(body_1.friction * body_2.friction),
                num_contacts,
                contacts: [contacts[0], contacts[1]],
            }
        }
    }
    pub fn update(
        &mut self,
        new_contacts: &[Contact],
        num_new_contacts: i32,
        world_context: &WorldContext,
    ) {
        let mut merged_contacts = [Contact::default(); 2];
        for i in 0..num_new_contacts as usize {
            let c_new = new_contacts[i];
            let mut k = -1;
            for j in 0..self.num_contacts {
                let c_old = self.contacts[j as usize];
                if c_new.feature.value == c_old.feature.value {
                    k = j;
                    break;
                }
            }
            if k > -1 {
                let c_old = self.contacts[k as usize];
                merged_contacts[i] = c_new;
                // Add the World::warm Start condition here
                if world_context.warm_starting {
                    merged_contacts[i].pn = c_old.pn;
                    merged_contacts[i].pt = c_old.pt;
                    merged_contacts[i].pnb = c_old.pnb;
                } else {
                    merged_contacts[i].pn = 0.0;
                    merged_contacts[i].pt = 0.0;
                    merged_contacts[i].pnb = 0.0;
                }
            } else {
                merged_contacts[i] = new_contacts[i];
            }
        }
        self.contacts = merged_contacts;
        self.num_contacts = num_new_contacts;
    }
    pub fn pre_step(&mut self, inv_dt: f32, world_context: &WorldContext) {
        let k_allowed_penetration = 0.0;
        let k_bias_factor = if world_context.position_correction {
            0.2
        } else {
            0.0
        };
        for i in 0..self.num_contacts as usize {
            let r1 = self.contacts[i].position - self.body1.position;
            let r2 = self.contacts[i].position - self.body2.position;

            // pre-compute normal mass , tangent mass, and bias
            let rn1 = r1.dot(self.contacts[i].normal);
            let rn2 = r2.dot(self.contacts[i].normal);
            let mut k_normal = self.body1.inv_mass + self.body2.inv_mass;
            k_normal += self.body1.inv_moi * (r1.dot(r2) - rn1 * rn2)
                + self.body2.inv_moi * (r2.dot(r2) - rn2 * rn2);
            self.contacts[i].mass_normal = 1.0 / k_normal;

            let tangent = (self.contacts[i].normal).cross(1.0);
            let rt1 = r1.dot(tangent);
            let rt2 = r2.dot(tangent);
            let mut k_tangent = self.body1.inv_mass + self.body2.inv_mass;
            k_tangent += self.body1.inv_moi * (r1.dot(r1) - rt1 * rt1)
                + self.body2.inv_moi * (r2.dot(r2) - rt2 * rt2);
            self.contacts[i].mass_tangent = 1.0 / k_tangent;

            self.contacts[i].bias = -k_bias_factor
                * inv_dt
                * f32::min(0.0, self.contacts[i].separation + k_allowed_penetration);
            if world_context.accumulate_impulse {
                let p =
                    self.contacts[i].normal * self.contacts[i].pn + tangent * self.contacts[i].pt;
                self.body1.velocity = self.body1.velocity - p * self.body1.inv_mass;
                self.body1.angular_velocity -= self.body1.inv_moi * r1.cross(p);

                self.body2.velocity = self.body2.velocity - p * self.body2.inv_mass;
                self.body2.angular_velocity -= self.body2.inv_moi * r2.cross(p);
            }
        }
    }
    pub fn apply_impulse(&mut self, world_context: &WorldContext) {
        for i in 0..self.num_contacts as usize {
            self.contacts[i].r1 = self.contacts[i].position - self.body1.position;
            self.contacts[i].r2 = self.contacts[i].position - self.body2.position;

            // Relative velocity at contact
            let dv = self.body2.velocity + self.body2.angular_velocity.cross(self.contacts[i].r2)
                - self.body1.velocity
                - self.body1.angular_velocity.cross(self.contacts[i].r1);

            // Compute normal impulse
            let vn = dv.dot(self.contacts[i].normal);
            let mut d_pn = self.contacts[i].mass_normal * (-vn + self.contacts[i].bias);

            if world_context.accumulate_impulse {
                // Clamp accumulated impulse
                let pn_0 = self.contacts[i].pn;
                self.contacts[i].pn = f32::max(pn_0 + d_pn, 0.0);
                d_pn = self.contacts[i].pn - pn_0;
            } else {
                d_pn = 0.0_f32.max(d_pn);
            }

            // Apply contact impulse
            let pn = self.contacts[i].normal * d_pn;

            self.body1.velocity = self.body1.velocity - pn * self.body1.inv_mass;
            self.body1.angular_velocity -= self.body1.inv_moi * self.contacts[i].r1.cross(pn);

            self.body2.velocity = self.body2.velocity + pn * self.body2.inv_mass;
            self.body2.angular_velocity += self.body2.inv_moi * self.contacts[i].r2.cross(pn);

            // Relative velocity at contact
            let dv = self.body2.velocity + self.body2.angular_velocity.cross(self.contacts[i].r2)
                - self.body1.velocity
                - self.body1.angular_velocity.cross(self.contacts[i].r1);

            let tangent = self.contacts[i].normal.cross(1.0);
            let vt = dv.dot(tangent);
            let mut d_pt = self.contacts[i].mass_tangent * -vt;
            if world_context.accumulate_impulse {
                // Compute friction impulse
                let max_pt = self.friction * self.contacts[i].pn;

                // Clamp friction
                let old_tangent_impulse = self.contacts[i].pt;
                self.contacts[i].pt = f32::clamp(old_tangent_impulse + d_pt, -max_pt, max_pt);
                d_pt = self.contacts[i].pt - old_tangent_impulse;
            } else {
                let max_pt = self.friction * d_pn;
                d_pt = f32::clamp(d_pt, -max_pt, max_pt);
            }

            // Apply contact impulse
            let pt = tangent * d_pt;

            self.body1.velocity = self.body1.velocity - pt * self.body1.inv_mass;
            self.body1.angular_velocity -= self.body1.inv_moi * self.contacts[i].r1.cross(pt);
            self.body2.velocity = self.body2.velocity + pt * self.body2.inv_mass;
            self.body2.angular_velocity -= self.body2.inv_moi * self.contacts[i].r2.cross(pt);
        }
    }
}
