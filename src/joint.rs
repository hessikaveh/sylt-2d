use crate::{
    body::Body,
    math_utils::{Cross, Mat2x2, Vec2},
    world::WorldContext,
};

#[derive(Default)]
pub struct Joint {
    p: Vec2, // accumulated impuls
    bias: Vec2,
    r1: Vec2,
    r2: Vec2,
    m: Mat2x2,
    bias_factor: f32,
    softness: f32,
    local_anchor_1: Vec2,
    local_anchor_2: Vec2,
    body_1: Body,
    body_2: Body,
}

impl Joint {
    pub fn new(body_1: Body, body_2: Body, anchor: Vec2) -> Self {
        let rot_trans_1 = Mat2x2::new_from_angle(body_1.rotation).transpose();
        let rot_trans_2 = Mat2x2::new_from_angle(body_2.rotation).transpose();
        let local_anchor_1 = rot_trans_1 * (anchor - body_1.position);
        let local_anchor_2 = rot_trans_2 * (anchor - body_2.position);

        Self {
            body_1,
            body_2,
            local_anchor_1,
            local_anchor_2,
            softness: 0.2,
            ..Default::default()
        }
    }

    pub fn pre_step(mut self, world_context: &WorldContext, inv_dt: f32) {
        let rot_1 = Mat2x2::new_from_angle(self.body_1.rotation);
        let rot_2 = Mat2x2::new_from_angle(self.body_2.rotation);

        self.r1 = rot_1 * self.local_anchor_1;
        self.r2 = rot_2 * self.local_anchor_2;

        // deltaV = deltaV0 + K * impulse
        // invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        let mut k1 = Mat2x2::default();
        k1.col1.x = self.body_1.inv_mass + self.body_2.inv_mass;
        k1.col2.x = 0.0;
        k1.col1.y = 0.0;
        k1.col2.y = self.body_1.inv_mass + self.body_2.inv_mass;

        let mut k2 = Mat2x2::default();
        k2.col1.x = self.body_1.inv_moi * self.r1.y * self.r1.y;
        k2.col2.x = -self.body_1.inv_moi * self.r1.x * self.r1.y;
        k2.col1.y = -self.body_1.inv_moi * self.r1.x * self.r1.y;
        k2.col2.y = self.body_1.inv_moi * self.r1.x * self.r1.x;

        let mut k3 = Mat2x2::default();
        k3.col1.x = self.body_2.inv_moi * self.r2.y * self.r2.y;
        k3.col2.x = -self.body_2.inv_moi * self.r2.x * self.r2.y;
        k3.col1.y = -self.body_2.inv_moi * self.r2.x * self.r2.y;
        k3.col2.y = self.body_2.inv_moi * self.r2.x * self.r2.x;

        let mut k = k1 + k2 + k3;
        k.col1.x += self.softness;
        k.col2.y += self.softness;

        self.m = k.invert();

        let p1 = self.body_1.position + self.r1;
        let p2 = self.body_2.position + self.r2;
        let dp = p2 - p1;

        if world_context.position_correction {
            self.bias = dp * inv_dt * -self.bias_factor;
        } else {
            self.bias = Vec2::default();
        }

        if world_context.warm_starting {
            self.body_1.velocity = self.body_1.velocity - self.p * self.body_1.inv_mass;
            self.body_1.angular_velocity -= self.body_1.inv_moi * self.r1.cross(self.p);
            self.body_2.velocity = self.body_2.velocity - self.p * self.body_2.inv_mass;
            self.body_2.angular_velocity -= self.body_2.inv_moi * self.r2.cross(self.p);
        } else {
            self.p = Vec2::default();
        }
    }
    pub fn apply_impulse(mut self) {
        let dv = self.body_2.velocity + self.body_2.angular_velocity.cross(self.r2)
            - self.body_1.velocity
            - self.body_1.angular_velocity.cross(self.r1);
        let impulse = self.m * (self.bias - dv - self.p * self.softness);
        self.body_1.velocity = self.body_1.velocity - impulse * self.body_1.inv_mass;
        self.body_1.angular_velocity -= self.body_1.inv_moi * self.r1.cross(impulse);

        self.body_2.velocity = self.body_2.velocity - impulse * self.body_2.inv_mass;
        self.body_2.angular_velocity -= self.body_2.inv_moi * self.r2.cross(impulse);

        self.p = self.p + impulse;
    }
}
