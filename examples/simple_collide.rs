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
        let mut box_a = Body::new(rect_a_half_size, 1.0);
        box_a.position = rect_a_position;
        box_a.rotation = rect_a_rotation;

        let mut box_b = Body::new(rect_b_half_size, 1.0);
        box_b.position = rect_b_position;
        box_b.rotation = rect_b_rotation;
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
