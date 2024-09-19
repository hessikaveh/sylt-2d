use crate::arbiter::Contact;
use crate::math_utils::{Mat2x2, Vec2};
// Define an enum for text styles
#[derive(Clone, Copy)]
pub enum TextStyle {
    Reset,
    Bold,
    Dim,
    Underline,
    Reversed,
    Hidden,
}

// Define an enum for text colors
#[derive(Clone, Copy)]
pub enum TextColor {
    Black,
    Red,
    Green,
    Yellow,
    Blue,
    Magenta,
    Cyan,
    White,
}

// Define a struct to hold color and style codes
#[derive(Clone, Copy)]
pub struct ColorStyle {
    pub text_color: TextColor,
    pub background_color: Option<TextColor>,
    pub style: Option<TextStyle>,
}

impl ColorStyle {
    // Method to get the ANSI escape code for the color and style
    fn color_style_to_ansi(&self) -> String {
        let mut codes = Vec::new();

        // Add text color code
        codes.push(match self.text_color {
            TextColor::Black => "30",
            TextColor::Red => "31",
            TextColor::Green => "32",
            TextColor::Yellow => "33",
            TextColor::Blue => "34",
            TextColor::Magenta => "35",
            TextColor::Cyan => "36",
            TextColor::White => "37",
        });

        // Add background color code if present
        if let Some(bg_color) = &self.background_color {
            codes.push(match bg_color {
                TextColor::Black => "40",
                TextColor::Red => "41",
                TextColor::Green => "42",
                TextColor::Yellow => "43",
                TextColor::Blue => "44",
                TextColor::Magenta => "45",
                TextColor::Cyan => "46",
                TextColor::White => "47",
            });
        }

        // Add text style code if present
        if let Some(style) = &self.style {
            codes.push(match style {
                TextStyle::Reset => "0",
                TextStyle::Bold => "1",
                TextStyle::Dim => "2",
                TextStyle::Underline => "4",
                TextStyle::Reversed => "7",
                TextStyle::Hidden => "8",
            });
        }

        // Join all codes with ';' and wrap with escape characters
        format!("\x1b[{}m", codes.join(";"))
    }
}

impl std::fmt::Display for ColorStyle {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self.color_style_to_ansi())
    }
}

impl Default for ColorStyle {
    fn default() -> Self {
        Self {
            text_color: TextColor::White,
            background_color: None,
            style: Some(TextStyle::Reset),
        }
    }
}

// Helper macros for creating color styles
macro_rules! color_style {
    ($text_color:expr, $bg_color:expr, $style:expr) => {
        ColorStyle {
            text_color: $text_color,
            background_color: $bg_color,
            style: $style,
        }
    };
}
pub fn get_styles() -> Vec<ColorStyle> {
    vec![
        ColorStyle {
            text_color: TextColor::Red,
            background_color: None,
            style: None,
        },
        ColorStyle {
            text_color: TextColor::Green,
            background_color: Some(TextColor::Black),
            style: Some(TextStyle::Bold),
        },
        ColorStyle {
            text_color: TextColor::Blue,
            background_color: Some(TextColor::Yellow),
            style: Some(TextStyle::Underline),
        },
        ColorStyle {
            text_color: TextColor::Magenta,
            background_color: None,
            style: Some(TextStyle::Reversed),
        },
        ColorStyle {
            text_color: TextColor::Cyan,
            background_color: Some(TextColor::Red),
            style: Some(TextStyle::Dim),
        },
        ColorStyle {
            text_color: TextColor::White,
            background_color: Some(TextColor::Blue),
            style: Some(TextStyle::Hidden),
        },
        ColorStyle {
            text_color: TextColor::Black,
            background_color: Some(TextColor::Green),
            style: Some(TextStyle::Bold),
        },
        ColorStyle {
            text_color: TextColor::Yellow,
            background_color: None,
            style: Some(TextStyle::Underline),
        },
    ]
}

/// Converts a Vec2 position to grid coordinates (for the ASCII grid).
fn pos_to_grid(pos: Vec2, grid_size: usize) -> (usize, usize) {
    let grid_origin = Vec2::new((grid_size / 2) as f32, (grid_size / 2) as f32); // Center of grid
    let grid_pos = grid_origin + pos;

    let x = grid_pos.x as usize;
    let y = grid_pos.y as usize;

    (x, y)
}

#[derive(Clone, Copy)]
pub struct StyledSymbol {
    symbol: char,
    style: ColorStyle,
}

// Define styles for drawing
const TICK_STYLE: ColorStyle = color_style!(TextColor::Cyan, None, Some(TextStyle::Bold));
const LABEL_STYLE: ColorStyle = color_style!(TextColor::Magenta, None, Some(TextStyle::Bold));
const COLLISION_STYLE: ColorStyle = color_style!(TextColor::Red, None, Some(TextStyle::Bold));

// Define a default styled symbol
impl Default for StyledSymbol {
    fn default() -> Self {
        Self {
            symbol: ' ',
            style: ColorStyle::default(),
        }
    }
}
fn create_styled_symbol(symbol: char, style: ColorStyle) -> StyledSymbol {
    StyledSymbol { symbol, style }
}
pub fn make_grid(grid_size: usize) -> Vec<Vec<StyledSymbol>> {
    let mut grid = vec![vec![StyledSymbol::default(); grid_size]; grid_size];

    // Draw ticks and axis labels on the grid
    for i in 0..grid_size {
        // Draw x-axis ticks
        grid[grid_size - 2][i] = create_styled_symbol('─', TICK_STYLE);

        // Draw y-axis ticks
        grid[i][1] = create_styled_symbol('│', TICK_STYLE);

        // Draw axis labels
        if i % 2 == 0 {
            let x_label = format!("{:2}", i);
            let y_label = format!("{:2}", i);

            // Set x-axis labels
            if let Some(ch) = x_label.chars().next() {
                grid[grid_size - 1][i] = create_styled_symbol(ch, LABEL_STYLE);
            }
            if let Some(ch) = x_label.chars().nth(1) {
                grid[grid_size - 3][i] = create_styled_symbol(ch, LABEL_STYLE);
            }

            // Set y-axis labels
            if let Some(ch) = y_label.chars().next() {
                grid[i][0] = create_styled_symbol(ch, LABEL_STYLE);
            }
            if let Some(ch) = y_label.chars().nth(1) {
                grid[i][2] = create_styled_symbol(ch, LABEL_STYLE);
            }
        }
    }

    // Draw axis arrows
    let arrow_style = LABEL_STYLE;
    let center_x = grid_size / 2;
    let center_y = grid_size / 2;

    grid[center_y][center_x] = create_styled_symbol('┬', arrow_style); // Arrow head at origin
    for i in center_x + 1..grid_size {
        grid[center_y][i] = create_styled_symbol('─', arrow_style); // Horizontal arrow line
    }
    for grid_y in grid.iter_mut().take(center_y) {
        grid_y[center_x] = create_styled_symbol('│', arrow_style); // Vertical arrow line
    }

    grid
}

pub fn add_line(
    grid: &mut Vec<Vec<StyledSymbol>>,
    start: Vec2,
    end: Vec2,
    symbol: char,
    style: ColorStyle,
) -> &Vec<Vec<StyledSymbol>> {
    let (x1, y1) = pos_to_grid(start, grid.len() / 2);
    let (x2, y2) = pos_to_grid(end, grid.len() / 2);

    let dx = x2 as i32 - x1 as i32;
    let dy = y2 as i32 - y1 as i32;

    let steps = dx.abs().max(dy.abs()) as usize;

    for i in 0..=steps {
        let t = i as f32 / steps as f32;
        let x = (x1 as f32 * (1.0 - t) + x2 as f32 * t).round() as usize;
        let y = (y1 as f32 * (1.0 - t) + y2 as f32 * t).round() as usize;

        if x < grid.len() && y < grid[0].len() {
            grid[y][x] = StyledSymbol { symbol, style };
        }
    }

    grid
}

pub fn add_point(
    grid: &mut Vec<Vec<StyledSymbol>>,
    position: Vec2,
    symbol: char,
    style: ColorStyle,
) -> &Vec<Vec<StyledSymbol>> {
    let (x, y) = pos_to_grid(position, grid.len() / 2);

    if x < grid.len() && y < grid[0].len() {
        grid[y][x] = StyledSymbol { symbol, style };
    }

    grid
}

pub fn add_box(
    grid: &mut Vec<Vec<StyledSymbol>>,
    pos: Vec2,
    h: Vec2,
    rot: f32,
    symbol: char,
    style: ColorStyle,
) {
    let rotation_matrix = Mat2x2::new_from_angle(rot);

    // Corners of the box (in local space)
    let corners = [
        Vec2::new(-h.x, -h.y), // Bottom-left
        Vec2::new(h.x, -h.y),  // Bottom-right
        Vec2::new(h.x, h.y),   // Top-right
        Vec2::new(-h.x, h.y),  // Top-left
    ];

    // Transform each corner to world space using rotation and position
    let world_corners: Vec<Vec2> = corners
        .iter()
        .map(|&corner| pos + rotation_matrix * corner)
        .collect();

    // Drawing the box
    for i in 0..4 {
        let (start, end) = (world_corners[i], world_corners[(i + 1) % 4]);
        add_line(grid, start, end, symbol, style);
    }

    // Mark the position of the box center
    let center = pos_to_grid(pos, grid.len() / 2);
    if center.0 < grid.len() && center.1 < grid[0].len() {
        grid[center.1][center.0] = StyledSymbol { symbol: 'O', style };
    }
}

pub fn draw_grid(grid: &mut Vec<Vec<StyledSymbol>>) {
    let reset_style = color_style!(TextColor::White, None, Some(TextStyle::Reset));

    for row in grid {
        for col in row {
            print!("{}{}{}", col.style, col.symbol, reset_style);
        }
        println!();
    }
}

pub fn draw_rectangle(
    grid: &mut [Vec<StyledSymbol>],
    pos: Vec2,
    size: Vec2,
    rotation: f32,
    style: ColorStyle,
) {
    let rotation_matrix = Mat2x2::new_from_angle(rotation);

    // Calculate the four corners of the rectangle in local space
    let half_size = size;
    let corners = [
        Vec2::new(-half_size.x, -half_size.y), // Top-left
        Vec2::new(half_size.x, -half_size.y),  // Top-right
        Vec2::new(half_size.x, half_size.y),   // Bottom-right
        Vec2::new(-half_size.x, half_size.y),  // Bottom-left
    ];

    // Rotate and translate corners to world space
    let transformed_corners: Vec<Vec2> = corners
        .iter()
        .map(|&corner| pos + rotation_matrix * corner)
        .collect();

    // Convert corners to grid coordinates
    let grid_corners: Vec<(usize, usize)> = transformed_corners
        .iter()
        .map(|&corner| pos_to_grid(corner, grid.len() / 2))
        .collect();

    // Draw the rectangle
    let (x0, y0) = grid_corners[0];
    let (x1, y1) = grid_corners[1];
    let (x2, y2) = grid_corners[2];
    let (x3, y3) = grid_corners[3];

    // Draw horizontal lines
    for x in x0.min(x1).min(x3).min(x2)..=x0.max(x1).max(x3).max(x2) {
        if y0 < grid.len() && y0 < grid[0].len() {
            grid[y0][x] = create_styled_symbol('─', style);
        }
        if y2 < grid.len() && y2 < grid[0].len() {
            grid[y2][x] = create_styled_symbol('─', style);
        }
    }

    // Draw vertical lines
    for y in y0.min(y1).min(y3).min(y2)..=y0.max(y1).max(y3).max(y2) {
        if x0 < grid.len() && x0 < grid[0].len() {
            grid[y][x0] = create_styled_symbol('│', style);
        }
        if x2 < grid.len() && x2 < grid[0].len() {
            grid[y][x2] = create_styled_symbol('│', style);
        }
    }

    // Draw corners
    grid[y0][x0] = create_styled_symbol('┌', style);
    grid[y0][x1] = create_styled_symbol('┐', style);
    grid[y2][x0] = create_styled_symbol('└', style);
    grid[y2][x1] = create_styled_symbol('┘', style);
}

pub fn draw_collision_result(grid: &mut Vec<Vec<StyledSymbol>>, contacts: &Vec<Contact>) {
    // Draw collision contacts
    for contact in contacts {
        add_point(grid, contact.position, 'C', COLLISION_STYLE);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_color_style_to_ansi() {
        let style = ColorStyle {
            text_color: TextColor::Red,
            background_color: Some(TextColor::Blue),
            style: Some(TextStyle::Bold),
        };

        let ansi_code = style.color_style_to_ansi();
        assert_eq!(ansi_code, "\x1b[31;44;1m", "Incorrect ANSI code generated");
    }

    #[test]
    fn test_draw_grid() {
        let mut grid = make_grid(20);

        draw_grid(&mut grid);

        // Visually inspect that the grid prints with styled symbols.
        // Alternatively, we can check that the correct symbols and ANSI codes are output, but this requires capturing stdout.
        // Asserting visual output is difficult, so this test is primarily for manual verification.
    }
}
