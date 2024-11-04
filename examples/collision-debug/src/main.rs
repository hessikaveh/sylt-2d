use nannou::prelude::*;
use nannou_egui::{self, egui, Egui};
use sylt_2d::{arbiter::Contact, body::Body, collide::collide, math_utils::Vec2};

fn main() {
    nannou::app(model).update(update).run();
}

struct EguiSettings {
    scale: f32,
    color: Srgb<u8>,
}

struct Model {
    _window: window::Id,
    demo_index: u32,
    egui: Egui,
    settings: EguiSettings,
    load_demo_flag: bool,
    contacts: Vec<Contact>,
    bodies: Vec<Body>,
    is_first_frame: bool,
}

fn model(app: &App) -> Model {
    let _window = app
        .new_window()
        .view(view)
        .raw_event(raw_window_event)
        .key_pressed(key_pressed)
        .build()
        .unwrap();
    let window = app.window(_window).unwrap();
    let egui = Egui::from_window(&window);
    Model {
        _window,
        demo_index: 0,
        egui,
        settings: EguiSettings {
            scale: 30.0,
            color: WHITE,
        },
        load_demo_flag: false,
        contacts: Vec::<Contact>::with_capacity(2),
        bodies: Vec::<Body>::with_capacity(2),
        is_first_frame: true,
    }
}

fn demo1(_model: &mut Model) {
    // Define boxes
    let pos_a = Vec2::new(10.0, 1.0);
    let pos_b = Vec2::new(15.0, 5.0);
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
    _model.bodies.push(box_a);
    _model.bodies.push(box_b);
    let _ = collide(&mut _model.contacts, &box_a, &box_b);
}

fn demo2(_model: &mut Model) {
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    _model.bodies.push(body1);
    let mut body2 = Body::new(Vec2::new(1.0, 1.0), 200.0);
    body2.position = Vec2::new(0.0, 4.0);
    _model.bodies.push(body2);
    let _ = collide(&mut _model.contacts, &body1, &body2);
}

fn demo3(_model: &mut Model) {
    // Define overlapping boxes
    let pos_a = Vec2::new(11.0, 3.0);
    let pos_b = Vec2::new(12., 2.);
    let box_a = Body {
        position: pos_a,
        rotation: 0.0,
        velocity: Vec2::new(0.0, 0.0),
        angular_velocity: 0.0,
        force: Vec2::new(0.0, 0.0),
        torque: 0.0,
        width: Vec2::new(2.0, 2.0),
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
        width: Vec2::new(2.0, 2.0),
        friction: 0.5,
        mass: 1.0,
        inv_mass: 1.0,
        moi: 1.0,
        inv_moi: 1.0,
    };
    _model.bodies.push(box_a);
    _model.bodies.push(box_b);
    let _ = collide(&mut _model.contacts, &box_a, &box_b);
}

fn demo4(_model: &mut Model) {
    // Define overlapping boxes at an angle
    let pos_a = Vec2::new(12.0, 0.0);
    let pos_b = Vec2::new(15.5, 1.0);
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
    _model.bodies.push(box_a);
    _model.bodies.push(box_b);
    let _ = collide(&mut _model.contacts, &box_a, &box_b);
}

fn demo5(_model: &mut Model) {
    // Define overlapping boxes at an angle
    let pos_a = Vec2::new(14.0, 2.0);
    let pos_b = Vec2::new(18.0, 2.0);
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
        rotation: 0.0,
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
    _model.bodies.push(box_a);
    _model.bodies.push(box_b);
    let _ = collide(&mut _model.contacts, &box_a, &box_b);
}

fn demo6(_model: &mut Model) {
    // Define boxes sharing an edge
    let pos_a = Vec2::new(1.0, 1.0);
    let pos_b = Vec2::new(5., 1.0);
    let box_a = Body {
        position: pos_a,
        rotation: 0.0,
        velocity: Vec2::new(0.0, 0.0),
        angular_velocity: 0.0,
        force: Vec2::new(0.0, 0.0),
        torque: 0.0,
        width: Vec2::new(2.0, 2.0),
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
        width: Vec2::new(2.0, 2.0),
        friction: 0.5,
        mass: 1.0,
        inv_mass: 1.0,
        moi: 1.0,
        inv_moi: 1.0,
    };
    _model.bodies.push(box_a);
    _model.bodies.push(box_b);
    let _ = collide(&mut _model.contacts, &box_a, &box_b);
}

fn demo7(_model: &mut Model) {}
fn update(_app: &App, _model: &mut Model, _update: Update) {
    if _model.is_first_frame {
        // Load the initial demo
        load_demo(_model);
        _model.is_first_frame = false;
    }
    if _model.load_demo_flag {
        load_demo(_model);
        _model.load_demo_flag = false;
    }

    let egui = &mut _model.egui;
    let settings = &mut _model.settings;

    egui.set_elapsed_time(_update.since_start);
    let ctx = egui.begin_frame();

    egui::Window::new("Settings").show(&ctx, |ui| {
        // Dropdown for selecting the demo
        ui.label("Select Demo:");
        egui::ComboBox::from_label("Demo Selection")
            .selected_text(format!("Demo {}", _model.demo_index + 1))
            .show_ui(ui, |ui| {
                for i in 0..7 {
                    ui.selectable_value(&mut _model.demo_index, i, format!("Demo {}", i + 1));
                }
            });

        // Button to load the selected demo
        if ui.button("Load Demo").clicked() {
            _model.load_demo_flag = true;
        }
        // Scale slider
        ui.label("Scale:");
        ui.add(egui::Slider::new(&mut settings.scale, 0.0..=1000.0));

        // Random color button
        let clicked = ui.button("Random color").clicked();

        if clicked {
            settings.color = rgb(random(), random(), random());
        }
    });
}

fn load_demo(model: &mut Model) {
    model.bodies.clear();
    model.contacts.clear();
    match model.demo_index {
        0 => demo1(model),
        1 => demo2(model),
        2 => demo3(model),
        3 => demo4(model),
        4 => demo5(model),
        5 => demo6(model),
        6 => demo7(model),
        _ => {}
    }
}
fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    // Let egui handle things like keyboard and mouse input.
    model.egui.handle_raw_event(event);
}

fn key_pressed(_app: &App, model: &mut Model, key: Key) {
    match key {
        Key::Right => {}
        Key::Left => {}
        _other_key => {}
    }
}

fn view(app: &App, _model: &Model, frame: Frame) {
    let draw = app.draw();
    let draw = draw.scale(_model.settings.scale);
    let settings = &_model.settings;
    draw.background().color(SLATEGREY);
    for (num, body) in _model.bodies.iter().enumerate() {
        draw.rect()
            .x_y(body.position.x, body.position.y)
            .w_h(body.width.x, body.width.y)
            .rotate(body.rotation)
            .color(if num == 0 { DARKSEAGREEN } else { ORCHID });
    }

    for contact in _model.contacts.iter() {
        match contact {
            Some(contact) => {
                draw.ellipse()
                    .x_y(contact.position.x, contact.position.y)
                    .radius(0.1)
                    .color(settings.color);
            }
            None => (),
        }
    }
    draw.to_frame(app, &frame).unwrap();
    _model.egui.draw_to_frame(&frame).unwrap();
}
