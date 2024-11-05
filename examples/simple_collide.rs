use sylt_2d::{
    body::Body,
    collide::collide,
    draw::{add_box, draw_collision_result, draw_grid, get_styles, make_grid},
    math_utils::Vec2,
};

use std::{
    io::{self, Write},
    thread,
    time::Duration,
};

fn main() {
    {
        // Create the grid for visualization
        let mut grid = make_grid(30);

        let rectangle_style_a = get_styles()[4]; // e.g., cyan, dim
        let rectangle_style_b = get_styles()[6]; // e.g., black, bold

        // Define the rectangles' positions, half-widths, and rotations
        let rect_a_position = Vec2::new(8.5, -0.5);
        let rect_a_half_size = Vec2::new(3.0, 2.0); // Width and height of Rectangle A
        let rect_a_rotation = 0.2; // Small rotation for Rectangle A

        let rect_b_position = Vec2::new(10.0, 1.0);
        let rect_b_half_size = Vec2::new(3.0, 2.0); // Width and height of Rectangle B
        let rect_b_rotation = 0.0; // No rotation for Rectangle B
                                   // Define boxes
        let box_a = Body {
            id: 1,
            position: rect_a_position,
            rotation: rect_a_rotation,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: rect_a_half_size,
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };
        let box_b = Body {
            id: 2,
            position: rect_b_position,
            rotation: rect_b_rotation,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            width: rect_b_half_size,
            friction: 0.5,
            mass: 1.0,
            inv_mass: 1.0,
            moi: 1.0,
            inv_moi: 1.0,
        };

        // Draw Rectangle A
        add_box(
            &mut grid,
            rect_a_position,
            rect_a_half_size,
            rect_a_rotation,
            'A',
            rectangle_style_a,
        );

        // Draw Rectangle B
        add_box(
            &mut grid,
            rect_b_position,
            rect_b_half_size,
            rect_b_rotation,
            'B',
            rectangle_style_b,
        );

        // Perform collision detection
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        println!("{:?}", contacts);
        println!("{:?}", num_contacts);
        println!("\x1b[2j");

        draw_collision_result(&mut grid, &contacts);

        // Display the grid
        draw_grid(&mut grid);

        let mut body1 = Body::new(Vec2::new(10.0, 2.0), f32::MAX);
        body1.position = Vec2::new(0.0, -0.05 * body1.width.y);

        let mut body2 = Body::new(Vec2::new(0.1, 0.1), 200.0);
        body2.position = Vec2::new(0.0, 0.4);
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        let mut grid = make_grid(100);
        // Draw Rectangle A
        add_box(
            &mut grid,
            body1.position,
            body1.width,
            body1.rotation,
            'A',
            rectangle_style_a,
        );

        // Draw Rectangle B
        add_box(
            &mut grid,
            body2.position,
            body2.width,
            body2.rotation,
            'B',
            rectangle_style_b,
        );

        println!("{:?}", contacts);
        println!("{:?}", num_contacts);
        println!("\x1b[2j");

        draw_collision_result(&mut grid, &contacts);

        // Display the grid
        draw_grid(&mut grid);

        let spinner = vec!['|', '/', '-', '\\'];
        let delay = Duration::from_millis(100);

        for _ in 0..1 {
            // Adjust the loop count to control duration
            for &frame in &spinner {
                print!("\r{} Loading...", frame);
                io::stdout().flush().unwrap(); // Flush to update the output immediately
                thread::sleep(delay);
            }
        }

        println!("\rDone!        "); // Clears the spinner when done
    }
}
