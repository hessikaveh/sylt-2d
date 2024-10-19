use std::usize;

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

pub struct ArbiterKey {
    body1: Body,
    body2: Body,
}

pub struct Arbiter {
    body1: Body,
    body2: Body,
    friction: f32,
    num_contacts: i32,
    contacts: [Contact; 2],
}

impl Arbiter {
    pub fn new(body_1: Body, body_2: Body) -> Self {
        let mut contacts = Vec::<Contact>::with_capacity(2);
        let num_contacts = collide(&mut contacts, &body_1, &body_2);
        Self {
            body1: body_1,
            body2: body_2,
            friction: f32::sqrt(body_1.friction * body_2.friction),
            num_contacts,
            contacts: [contacts[0], contacts[1]],
        }
    }
    pub fn update(&mut self, new_contacts: &[Contact], num_new_contacts: i32) {
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
                merged_contacts[i].pn = c_old.pn;
                merged_contacts[i].pt = c_old.pt;
                merged_contacts[i].pnb = c_old.pnb;
            } else {
                merged_contacts[i] = new_contacts[i];
            }
        }
        self.contacts = merged_contacts;
        self.num_contacts = num_new_contacts;
    }
    pub fn pre_step() {}
}
