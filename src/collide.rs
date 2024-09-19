use crate::arbiter::{Contact, EdgeNumbers, Edges, FeaturePair};
use crate::body::Body;
use crate::math_utils::{Mat2x2, Vec2};

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

/**
This module provides functions for detecting collisions between two axis-aligned bounding boxes (AABBs).

## Data Structures

### `Axis`
An enumeration representing the possible axes along which collision detection is performed:
- `FaceAX`: Face of box A along the X-axis.
- `FaceAY`: Face of box A along the Y-axis.
- `FaceBX`: Face of box B along the X-axis.
- `FaceBY`: Face of box B along the Y-axis.

### `ClipVertex`
A struct representing a vertex on the edge of a box used for clipping:
- `fp`: `FeaturePair` containing information about the edges.
- `v`: The vertex position in 2D space.


### `clip_segment_to_line`
Clips a line segment against a clipping plane to determine which vertices are inside or outside the clipping plane.

#### Parameters
- `v_out`: Output array for the clipped vertices.
- `v_in`: Input array of vertices for the line segment.
- `normal`: Normal vector of the clipping plane.
- `offset`: Offset from the origin to the clipping plane.
- `clip_edge`: Edge of the clipping plane.

#### Returns
- `usize`: The number of vertices in `v_out`.

### `compute_incident_edge`
Computes the vertices of the edge of a box that is incident to the face of another box, based on the given normal.

#### Parameters
- `h`: Half-extents of the box.
- `pos`: Position of the box.
- `rot`: Rotation matrix of the box.
- `normal`: Normal vector from the reference box.

#### Returns
- `[ClipVertex; 2]`: An array containing the vertices of the incident edge.

### `collide`
Main function for detecting collisions between two boxes and computing contact points.

#### Parameters
- `contacts`: A mutable vector to store contact points.
- `body_a`: The first box to be tested for collision.
- `body_b`: The second box to be tested for collision.

#### Returns
- `i32`: The number of contact points generated.

#### Description
- **Transform Box Positions and Orientations**: Converts box positions and orientations to a common frame of reference.
- **Compute Face Projections**: Calculates how much each box overlaps along the axes of potential separation.
- **Determine Axis of Minimum Penetration**: Identifies the axis that causes the least penetration, i.e., the most likely direction of collision.
- **Compute Incident Edge**: Computes the vertices of the incident edge based on the detected axis.
- **Clip Against Edges**: Finds the intersection points of the incident edge with the clipping edges of the other box.
- **Generate Contacts**: Creates `Contact` structures for each valid contact point and adds them to the contacts vector.

### Constants
- `RELATIVE_TOL`: Relative tolerance for collision detection to handle floating-point precision issues.
- `ABSOLUTE_TOL`: Absolute tolerance for collision detection.
*/

#[derive(PartialEq)]
enum Axis {
    FaceAX,
    FaceAY,
    FaceBX,
    FaceBY,
}

#[derive(Clone, Copy)]
pub struct ClipVertex {
    fp: FeaturePair,
    pub v: Vec2,
}

impl Default for ClipVertex {
    fn default() -> Self {
        Self {
            fp: FeaturePair::new(Edges::default(), 0),
            v: Vec2::new(0.0, 0.0),
        }
    }
}

fn flip(fp: &mut FeaturePair) {
    std::mem::swap(&mut fp.edges.in_edge_1, &mut fp.edges.in_edge_2);
    std::mem::swap(&mut fp.edges.out_edge_1, &mut fp.edges.out_edge_2);
}

fn clip_segment_to_line(
    v_out: &mut [ClipVertex; 2],
    v_in: [ClipVertex; 2],
    normal: &Vec2,
    offset: f32,
    clip_edge: EdgeNumbers,
) -> usize {
    let mut num_out: usize = 0;

    // Calculate the distance of end points to the line
    let distance_0 = normal.dot(v_in[0].v) - offset;
    let distance_1 = normal.dot(v_in[1].v) - offset;

    // If the points are behind the plane
    if distance_0 <= 0.0 {
        v_out[num_out] = v_in[0];
        num_out += 1;
    }
    if distance_1 <= 0.0 {
        v_out[num_out] = v_in[1];
        num_out += 1;
    }

    // If the points are on different sides of the plane
    if distance_0 * distance_1 < 0.0 {
        // Find intersection point of edge and plane
        let interp = distance_0 / (distance_0 - distance_1);
        v_out[num_out].v = v_in[0].v + (v_in[1].v - v_in[0].v) * interp;

        // Set feature points based on which point is in front of the plane
        if distance_0 > 0.0 {
            v_out[num_out].fp = v_in[0].fp;
            v_out[num_out].fp.edges.in_edge_1 = clip_edge;
            v_out[num_out].fp.edges.in_edge_2 = EdgeNumbers::NoEdge;
        } else {
            v_out[num_out].fp = v_in[1].fp;
            v_out[num_out].fp.edges.out_edge_1 = clip_edge;
            v_out[num_out].fp.edges.out_edge_2 = EdgeNumbers::NoEdge;
        }
        num_out += 1;
    }

    num_out
}

fn compute_incident_edge(h: &Vec2, pos: &Vec2, rot: &Mat2x2, normal: &Vec2) -> [ClipVertex; 2] {
    // The normal is from the reference box. Convert it
    // to the incident boxe's frame and flip sign.
    let rot_t = rot.transpose();
    let n = -(rot_t * *normal);
    let abs_n = n.abs();
    let mut c1 = ClipVertex::default();
    let mut c2 = ClipVertex::default();

    if abs_n.x > abs_n.y {
        if n.x.signum() > 0.0 {
            c1.v = Vec2::new(h.x, -h.y);
            c1.fp.edges.in_edge_2 = EdgeNumbers::Edge3;
            c1.fp.edges.out_edge_2 = EdgeNumbers::Edge4;

            c2.v = Vec2::new(h.x, h.y);
            c2.fp.edges.in_edge_2 = EdgeNumbers::Edge4;
            c2.fp.edges.out_edge_2 = EdgeNumbers::Edge1;
        } else {
            c1.v = Vec2::new(-h.x, h.y);
            c1.fp.edges.in_edge_2 = EdgeNumbers::Edge1;
            c1.fp.edges.out_edge_2 = EdgeNumbers::Edge2;

            c2.v = Vec2::new(-h.x, -h.y);
            c2.fp.edges.in_edge_2 = EdgeNumbers::Edge2;
            c2.fp.edges.out_edge_2 = EdgeNumbers::Edge3;
        }
    } else if n.y.signum() > 0.0 {
        c1.v = Vec2::new(h.x, h.y);
        c1.fp.edges.in_edge_2 = EdgeNumbers::Edge4;
        c1.fp.edges.out_edge_2 = EdgeNumbers::Edge1;

        c2.v = Vec2::new(-h.x, h.y);
        c2.fp.edges.in_edge_2 = EdgeNumbers::Edge1;
        c2.fp.edges.out_edge_2 = EdgeNumbers::Edge2;
    } else {
        c1.v = Vec2::new(-h.x, -h.y);
        c1.fp.edges.in_edge_2 = EdgeNumbers::Edge2;
        c1.fp.edges.out_edge_2 = EdgeNumbers::Edge3;

        c2.v = Vec2::new(h.x, -h.y);
        c2.fp.edges.in_edge_2 = EdgeNumbers::Edge3;
        c2.fp.edges.out_edge_2 = EdgeNumbers::Edge4;
    }

    c1.v = *pos + (*rot * c1.v);
    c2.v = *pos + (*rot * c2.v);
    [c1, c2]
}

pub fn collide(contacts: &mut Vec<Contact>, body_a: &Body, body_b: &Body) -> i32 {
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

    let front_normal;
    let side_normal;
    let neg_side;
    let pos_side;
    let neg_edge;
    let pos_edge;
    let front;
    let side;

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
    let mut clip_points1 = [ClipVertex::default(), ClipVertex::default()];
    let mut clip_points2 = [ClipVertex::default(), ClipVertex::default()];

    let mut np = clip_segment_to_line(
        &mut clip_points1,
        incident_edge,
        &(-side_normal),
        neg_side,
        neg_edge,
    );
    if np < 2 {
        return 0;
    };

    np = clip_segment_to_line(
        &mut clip_points2,
        clip_points1,
        &(-side_normal),
        neg_side,
        neg_edge,
    );
    if np < 2 {
        return 0;
    };
    let mut num_contacts = 0;

    for clip_point in &mut clip_points2 {
        let separation = front_normal.dot(clip_point.v) - front;

        if separation <= 0.0 {
            if axis == Axis::FaceBX || axis == Axis::FaceBY {
                flip(&mut clip_point.fp);
            }
            let contact = Contact {
                separation,
                normal,
                position: clip_point.v - front_normal * separation,
                feature: clip_point.fp,
                ..Contact::default()
            };
            contacts.push(contact);
            num_contacts += 1;
        }
    }
    num_contacts
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::draw::{add_box, draw_collision_result, draw_grid, get_styles, make_grid};
    use crate::math_utils::Vec2;

    #[test]
    fn test_no_overlap() {
        let styles = get_styles();
        let mut grid = make_grid(20);

        // Define boxes
        let pos_a = Vec2::new(1.0, 1.0);
        let pos_b = Vec2::new(5.0, 5.0);
        let box_a = Body {
            position: pos_a,
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: Vec2::new(1.0, 1.0),
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };
        let box_b = Body {
            position: pos_b,
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: Vec2::new(1.0, 1.0),
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };

        // Draw the boxes
        add_box(
            &mut grid,
            pos_a,
            box_a.width * 0.5,
            box_a.rotation,
            'A',
            styles[4],
        );
        add_box(
            &mut grid,
            pos_b,
            box_b.width * 0.5,
            box_b.rotation,
            'B',
            styles[6],
        );

        // Perform collision detection
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        println!("{:?}", contacts);
        draw_collision_result(&mut grid, &box_a, &box_b, &contacts);
        // Draw the grid
        draw_grid(&mut grid);

        // Assertions to ensure no intersection
        assert_eq!(
            num_contacts, 0,
            "Expected no contacts, but found {}",
            num_contacts
        );
    }

    #[test]
    fn test_overlap() {
        let styles = get_styles();
        let mut grid = make_grid(20);

        // Define overlapping boxes
        let pos_a = Vec2::new(1.0, 1.0);
        let pos_b = Vec2::new(1.5, 1.5);
        let box_a = Body {
            position: pos_a,
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: Vec2::new(1.0, 1.0),
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };
        let box_b = Body {
            position: pos_b,
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: Vec2::new(1.0, 1.0),
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };

        // Draw the boxes
        add_box(
            &mut grid,
            pos_a,
            box_a.width * 0.5,
            box_a.rotation,
            'A',
            styles[4],
        );
        add_box(
            &mut grid,
            pos_b,
            box_b.width * 0.5,
            box_b.rotation,
            'B',
            styles[6],
        );

        // Perform collision detection
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        println!("{:?}", contacts);
        draw_collision_result(&mut grid, &box_a, &box_b, &contacts);
        // Draw the grid
        draw_grid(&mut grid);

        // Assertions to ensure intersection
        assert!(
            num_contacts > 0,
            "Expected contacts, but found {}",
            num_contacts
        );
    }

    #[test]
    fn test_overlap_at_angle() {
        let styles = get_styles();
        let mut grid = make_grid(20);

        // Define overlapping boxes at an angle
        let pos_a = Vec2::new(0.0, 0.0);
        let pos_b = Vec2::new(3.5, 1.0);
        let box_a = Body {
            position: pos_a,
            rotation: 45.0_f32.to_radians(),
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: Vec2::new(4.0, 4.0),
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };
        let box_b = Body {
            position: pos_b,
            rotation: -45.0_f32.to_radians(),
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: Vec2::new(4.0, 4.0),
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };

        // Draw the boxes
        add_box(
            &mut grid,
            pos_a,
            box_a.width * 0.5,
            box_a.rotation,
            'A',
            styles[4],
        );
        add_box(
            &mut grid,
            pos_b,
            box_b.width * 0.5,
            box_b.rotation,
            'B',
            styles[6],
        );

        // Perform collision detection
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        println!("{:?}", contacts);
        draw_collision_result(&mut grid, &box_a, &box_b, &contacts);
        // Draw the grid
        draw_grid(&mut grid);

        // Assertions to ensure intersection
        assert!(
            num_contacts > 0,
            "Expected contacts, but found {}",
            num_contacts
        );
    }
    #[test]
    fn test_edge_case() {
        let styles = get_styles();
        let mut grid = make_grid(20);

        // Define boxes sharing an edge
        let pos_a = Vec2::new(1.0, 1.0);
        let pos_b = Vec2::new(1.5, 1.0);
        let box_a = Body {
            position: pos_a,
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: Vec2::new(1.0, 1.0),
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };
        let box_b = Body {
            position: pos_b,
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: Vec2::new(1.0, 1.0),
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };

        // Draw the boxes
        add_box(
            &mut grid,
            pos_a,
            box_a.width * 0.5,
            box_a.rotation,
            'A',
            styles[4],
        );
        add_box(
            &mut grid,
            pos_b,
            box_b.width * 0.5,
            box_b.rotation,
            'B',
            styles[6],
        );

        // Perform collision detection
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        println!("{:?}", contacts);
        draw_collision_result(&mut grid, &box_a, &box_b, &contacts);
        // Draw the grid
        draw_grid(&mut grid);

        // Assertions to ensure edge case is handled
        assert!(
            num_contacts > 0,
            "Expected contacts, but found {}",
            num_contacts
        );
    }
}
