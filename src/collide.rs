use crate::arbiter::Contact;
use crate::body::Body;
use crate::math_utils::{Mat2x2, Vec2};

enum Axis {
    FaceAX,
    FaceAY,
    FaceBX,
    FaceBY,
}

enum EdgeNumbers {
    NoEdge = 0,
    Edge1,
    Edge2,
    Edge3,
    Edge4,
}

struct ClipVertex {}

fn compute_incident_edge(h: &Vec2, pos: &Vec2, rot: &Mat2x2, normal: &Vec2) -> [ClipVertex; 2] {
    let c1 = ClipVertex {};
    let c2 = ClipVertex {};
    [c1, c2]
}

pub fn collide(contact: &Contact, body_a: &Body, body_b: &Body) -> i32 {
    let h_a = body_a.width * 0.5;
    let h_b = body_b.width * 0.5;

    let pos_a = body_a.position;
    let pos_b = body_b.position;
    let rot_a = Mat2x2::new_from_angle(body_a.rotation);
    let rot_b = Mat2x2::new_from_angle(body_b.rotation);

    let rot_a_t = rot_a.transpose();
    let rot_b_t = rot_b.transpose();

    let d_p = pos_b - pos_a;
    let d_a = rot_a_t * d_p;
    let d_b = rot_b_t * d_p;

    let c = rot_a_t * rot_b;
    let abs_c = c.abs();
    let abs_c_t = c.abs().transpose();

    let face_a = d_a.abs() - h_a - abs_c * h_b;
    if face_a.x > 0.0 || face_a.y > 0.0 {
        return 0;
    };
    let face_b = d_b.abs() - h_b - abs_c_t * h_a;
    if face_b.x > 0.0 || face_b.y > 0.0 {
        return 0;
    };

    let mut axis = Axis::FaceAX;
    let mut separation = face_a.x;
    let mut normal = if d_a.x > 0.0 { rot_a.col1 } else { -rot_a.col1 };

    // Tolerances
    const RELATIVE_TOL: f32 = 0.95;
    const ABSOLUTE_TOL: f32 = 0.01;

    // Box A faces
    if face_a.y > RELATIVE_TOL * separation + ABSOLUTE_TOL * h_a.y {
        axis = Axis::FaceAY;
        separation = face_a.y;
        normal = if d_a.y > 0.0 { rot_a.col2 } else { -rot_a.col2 };
    }

    // Box B faces
    if face_b.x > RELATIVE_TOL * separation + ABSOLUTE_TOL * h_b.x {
        axis = Axis::FaceBX;
        separation = face_b.x;
        normal = if d_b.x > 0.0 { rot_b.col1 } else { -rot_b.col1 };
    }

    if face_b.y > RELATIVE_TOL * separation + ABSOLUTE_TOL * h_b.y {
        axis = Axis::FaceBY;
        separation = face_b.y;
        normal = if d_b.y > 0.0 { rot_b.col2 } else { -rot_b.col2 };
    }

    let mut front_normal;
    let mut side_normal;
    let mut neg_side;
    let mut pos_side;
    let mut neg_edge;
    let mut pos_edge;
    let mut front;
    let mut side;

    let incident_edge = match axis {
        Axis::FaceAX => {
            front_normal = normal;
            front = front_normal.dot(pos_a) + h_a.x;
            side_normal = rot_a.col2;
            side = pos_a.dot(side_normal);
            neg_side = -side + h_a.y;
            pos_side = side + h_a.y;
            neg_edge = EdgeNumbers::Edge3;
            pos_edge = EdgeNumbers::Edge1;
            compute_incident_edge(&h_b, &pos_b, &rot_b, &front_normal)
        }
        Axis::FaceAY => {
            front_normal = normal;
            front = front_normal.dot(pos_a) + h_a.y;
            side_normal = rot_a.col1;
            side = pos_a.dot(side_normal);
            neg_side = -side + h_a.x;
            pos_side = side + h_a.x;
            neg_edge = EdgeNumbers::Edge2;
            pos_edge = EdgeNumbers::Edge4;
            compute_incident_edge(&h_b, &pos_b, &rot_b, &front_normal)
        }
        Axis::FaceBX => {
            front_normal = -normal;
            front = front_normal.dot(pos_b) + h_b.x;
            side_normal = rot_b.col2;
            side = pos_b.dot(side_normal);
            neg_side = -side + h_b.y;
            pos_side = side + h_b.y;
            neg_edge = EdgeNumbers::Edge3;
            pos_edge = EdgeNumbers::Edge1;
            compute_incident_edge(&h_a, &pos_a, &rot_a, &front_normal)
        }
        Axis::FaceBY => {
            front_normal = -normal;
            front = front_normal.dot(pos_b) + h_b.y;
            side_normal = rot_b.col1;
            side = pos_b.dot(side_normal);
            neg_side = -side + h_b.x;
            pos_side = side + h_b.x;
            neg_edge = EdgeNumbers::Edge2;
            pos_edge = EdgeNumbers::Edge4;
            compute_incident_edge(&h_a, &pos_a, &rot_a, &front_normal)
        }
    };

    1
}
