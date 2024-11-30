use crate::body::Shape;
use crate::collide_polygon::collide_polygons;
use crate::math_utils::Cross;
use crate::world::WorldContext;
use crate::{body::Body, collide::collide, math_utils::Vec2};
use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;

#[derive(Debug)]
pub enum ArbiterErrors {
    NoOldContactFound,
    NoNewContactFound,
}

impl fmt::Display for ArbiterErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ArbiterErrors::NoOldContactFound => write!(f, "No old contacts found."),
            ArbiterErrors::NoNewContactFound => write!(f, "No new contacts found."),
        }
    }
}

impl std::error::Error for ArbiterErrors {}

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

#[derive(Debug, Default, Clone, Copy)]
pub struct FeaturePair {
    pub edges: Edges,
    pub value: i32,
}

impl FeaturePair {
    pub fn new(edges: Edges, value: i32) -> Self {
        Self { edges, value }
    }
}

pub type Contact = Option<ContactInfo>;

#[derive(Debug, Default, Clone, Copy)]
pub struct ContactInfo {
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

#[derive(Debug, Eq, Hash, PartialEq)]
pub struct ArbiterKey {
    body1_id: usize,
    body2_id: usize,
}

impl ArbiterKey {
    pub fn new(body_1: &Body, body_2: &Body) -> Self {
        if body_1.id < body_2.id {
            Self {
                body1_id: body_1.id,
                body2_id: body_2.id,
            }
        } else {
            Self {
                body1_id: body_2.id,
                body2_id: body_1.id,
            }
        }
    }
}

#[derive(Debug)]
pub struct Arbiter {
    body1: Rc<RefCell<Body>>,
    body2: Rc<RefCell<Body>>,
    friction: f32,
    pub num_contacts: i32,
    pub contacts: Vec<Contact>,
}

impl Arbiter {
    pub fn new(body_1: Rc<RefCell<Body>>, body_2: Rc<RefCell<Body>>) -> Self {
        let mut contacts = Vec::<Contact>::with_capacity(2);

        if body_1.borrow().id > body_2.borrow().id {
            body_1.swap(&body_2);
        };

        let num_contacts = match (body_1.borrow().shape, body_2.borrow().shape) {
            (Shape::Box, Shape::Box) => collide(&mut contacts, &body_1.borrow(), &body_2.borrow()),
            _ => collide_polygons(&mut contacts, &body_1.borrow(), &body_2.borrow()),
        };
        let friction = f32::sqrt(body_1.borrow().friction * body_2.borrow().friction);
        Self {
            body1: body_1,
            body2: body_2,
            friction,
            num_contacts,
            contacts,
        }
    }
    pub fn update(
        &mut self,
        new_contacts: &[Contact],
        num_new_contacts: i32,
        world_context: &WorldContext,
    ) -> Result<(), ArbiterErrors> {
        let mut merged_contacts = Vec::<Contact>::with_capacity(2);

        for new_contact in new_contacts.iter() {
            let mut k = -1;
            let new_contact_feature = new_contact.map(|contact| contact.feature.value);
            if new_contact_feature.is_some() {
                for (j, contact) in self.contacts.iter().enumerate() {
                    if Some(contact.map(|contact| contact.feature.value))
                        == Some(new_contact_feature)
                    {
                        k = j as i32;
                        break;
                    }
                }
                if k > -1 {
                    let c_old =
                        self.contacts[k as usize].ok_or(ArbiterErrors::NoOldContactFound)?;
                    let mut new_contact_ = new_contact.ok_or(ArbiterErrors::NoNewContactFound)?;
                    if world_context.warm_starting {
                        new_contact_.pn = c_old.pn;
                        new_contact_.pt = c_old.pt;
                        new_contact_.pnb = c_old.pnb;
                    } else {
                        new_contact_.pn = 0.0;
                        new_contact_.pt = 0.0;
                        new_contact_.pnb = 0.0;
                    }

                    merged_contacts.push(Some(new_contact_));
                } else {
                    merged_contacts.push(*new_contact);
                }
            }
        }

        self.contacts = merged_contacts;
        self.num_contacts = num_new_contacts;
        Ok(())
    }
    pub fn pre_step(&mut self, inv_dt: f32, world_context: &WorldContext) {
        let k_allowed_penetration = 0.01;
        let k_bias_factor = if world_context.position_correction {
            0.2
        } else {
            0.0
        };
        let mut body1 = self.body1.borrow_mut();
        let mut body2 = self.body2.borrow_mut();
        for contact in self.contacts.iter_mut() {
            match contact {
                Some(contact) => {
                    let r1 = contact.position - body1.position;
                    let r2 = contact.position - body2.position;

                    // pre-compute normal mass , tangent mass, and bias
                    let rn1 = r1.dot(contact.normal);
                    let rn2 = r2.dot(contact.normal);
                    let mut k_normal = body1.inv_mass + body2.inv_mass;
                    k_normal += body1.inv_moi * (r1.dot(r1) - rn1 * rn1)
                        + body2.inv_moi * (r2.dot(r2) - rn2 * rn2);
                    contact.mass_normal = 1.0 / k_normal;

                    let tangent = (contact.normal).cross(1.0);
                    let rt1 = r1.dot(tangent);
                    let rt2 = r2.dot(tangent);
                    let mut k_tangent = body1.inv_mass + body2.inv_mass;
                    k_tangent += body1.inv_moi * (r1.dot(r1) - rt1 * rt1)
                        + body2.inv_moi * (r2.dot(r2) - rt2 * rt2);
                    contact.mass_tangent = 1.0 / k_tangent;

                    contact.bias = -k_bias_factor
                        * inv_dt
                        * f32::min(0.0, contact.separation + k_allowed_penetration);
                    if world_context.accumulate_impulse {
                        let p = contact.normal * contact.pn + tangent * contact.pt;
                        body1.velocity = body1.velocity - p * body1.inv_mass;
                        body1.angular_velocity -= body1.inv_moi * r1.cross(p);

                        body2.velocity = body2.velocity + p * body2.inv_mass;
                        body2.angular_velocity += body2.inv_moi * r2.cross(p);
                    };
                }
                None => (),
            }
        }
    }
    pub fn apply_impulse(&mut self, world_context: &WorldContext) {
        let mut body1 = self.body1.borrow_mut();
        let mut body2 = self.body2.borrow_mut();

        for contact in self.contacts.iter_mut() {
            match contact {
                Some(contact) => {
                    contact.r1 = contact.position - body1.position;
                    contact.r2 = contact.position - body2.position;

                    // Relative velocity at contact
                    let dv = body2.velocity + body2.angular_velocity.cross(contact.r2)
                        - body1.velocity
                        - body1.angular_velocity.cross(contact.r1);

                    // Compute normal impulse
                    let vn = dv.dot(contact.normal);
                    let mut d_pn = contact.mass_normal * (-vn + contact.bias);

                    if world_context.accumulate_impulse {
                        // Clamp accumulated impulse
                        let pn_0 = contact.pn;
                        contact.pn = f32::max(pn_0 + d_pn, 0.0);
                        d_pn = contact.pn - pn_0;
                    } else {
                        d_pn = 0.0_f32.max(d_pn);
                    };

                    // Apply contact impulse
                    let pn = contact.normal * d_pn;

                    body1.velocity = body1.velocity - pn * body1.inv_mass;
                    body1.angular_velocity -= body1.inv_moi * contact.r1.cross(pn);

                    body2.velocity = body2.velocity + pn * body2.inv_mass;
                    body2.angular_velocity += body2.inv_moi * contact.r2.cross(pn);

                    // Relative velocity at contact
                    let dv = body2.velocity + body2.angular_velocity.cross(contact.r2)
                        - body1.velocity
                        - body1.angular_velocity.cross(contact.r1);

                    let tangent = contact.normal.cross(1.0);
                    let vt = dv.dot(tangent);
                    let mut d_pt = contact.mass_tangent * -vt;
                    if world_context.accumulate_impulse {
                        // Compute friction impulse
                        let max_pt = self.friction * contact.pn;

                        // Clamp friction
                        let old_tangent_impulse = contact.pt;
                        contact.pt = f32::clamp(old_tangent_impulse + d_pt, -max_pt, max_pt);
                        d_pt = contact.pt - old_tangent_impulse;
                    } else {
                        let max_pt = self.friction * d_pn;
                        d_pt = f32::clamp(d_pt, -max_pt, max_pt);
                    };

                    // Apply contact impulse
                    let pt = tangent * d_pt;

                    body1.velocity = body1.velocity - pt * body1.inv_mass;
                    body1.angular_velocity -= body1.inv_moi * contact.r1.cross(pt);
                    body2.velocity = body2.velocity + pt * body2.inv_mass;
                    body2.angular_velocity += body2.inv_moi * contact.r2.cross(pt);
                }
                None => (),
            }
        }
    }
}
