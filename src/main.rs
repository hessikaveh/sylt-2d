use sylt_2d::{
    body::Body,
    collide::collide,
    draw::{
        add_box, add_line, draw_collision_result, draw_grid, draw_rectangle, get_styles, make_grid,
    },
    math_utils::Vec2,
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

        draw_collision_result(&mut grid, &box_a, &box_b, &contacts);

        // Display the grid
        draw_grid(&mut grid);
    }
}
