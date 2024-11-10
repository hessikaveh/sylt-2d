use nannou::prelude::*;
use nannou_egui::{self, egui, Egui};
use sylt_2d::body::Body;
use sylt_2d::joint::Joint;
use sylt_2d::math_utils::{Mat2x2, Vec2};
use sylt_2d::world::World;
fn main() {
    nannou::app(model).update(update).run();
}
const ITERATIONS: u32 = 100;

struct EguiSettings {
    scale: f32,
    color: Srgb<u8>,
}

struct Model {
    _window: window::Id,
    time_step: f32,
    demo_index: u32,
    world: World,
    bomb: bool,
    egui: Egui,
    settings: EguiSettings,
    is_first_frame: bool,
    load_demo_flag: bool,
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
    let world = World::new(Vec2::new(0.0, -10.0), ITERATIONS);
    Model {
        _window,
        world,
        demo_index: 0,
        bomb: false,
        time_step: 1.0 / 60.0,
        egui,
        settings: EguiSettings {
            scale: 18.0,
            color: WHITE,
        },
        is_first_frame: true,
        load_demo_flag: false,
    }
}

fn launch_bomb(model: &mut Model) {
    let mut bomb = Body::new(Vec2::new(1.0, 1.0), 50.0);
    bomb.friction = 0.2;
    bomb.position = Vec2::new(random_range(-15.0, 15.0), 15.0);
    bomb.rotation = random_range(-1.5, 1.5);
    bomb.velocity = bomb.position * -1.5;
    bomb.angular_velocity = random_range(-20.0, 20.0);
    model.world.add_body(bomb);
}

fn demo1(_model: &mut Model) {
    // Single box
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    _model.world.add_body(body1);

    let mut body2 = Body::new(Vec2::new(1.0, 1.0), 200.0);
    body2.position = Vec2::new(0.0, 4.0);
    _model.world.add_body(body2);
}

fn demo2(model: &mut Model) {
    // Simple Pendulum
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.friction = 0.2;
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    body1.rotation = 0.0;
    model.world.add_body(body1);

    let mut body2 = Body::new(Vec2::new(1.0, 1.0), 100.0);
    body2.friction = 0.2;
    body2.position = Vec2::new(9.0, 11.0);
    body2.rotation = 0.0;
    model.world.add_body(body2);

    let joint = Joint::new(body1, body2, Vec2::new(0.0, 11.0), &model.world);
    model.world.add_joint(joint);
}

fn demo3(model: &mut Model) {
    let friction_values = [0.75, 0.5, 0.35, 0.1, 0.0];

    let mut body = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body.position = Vec2::new(0.0, -0.5 * body.width.y);
    model.world.add_body(body);

    let mut body2 = Body::new(Vec2::new(13.0, 0.25), f32::MAX);
    body2.position = Vec2::new(-2.0, 11.0);
    body2.rotation = -0.25;
    model.world.add_body(body2);

    // Additional bodies with varying frictions
    for (i, &friction) in friction_values.iter().enumerate() {
        let mut body = Body::new(Vec2::new(0.5, 0.5), 25.0);
        body.friction = friction;
        body.position = Vec2::new(-7.5 + 2.0 * i as f32, 14.0);
        model.world.add_body(body);
    }
}

fn demo4(model: &mut Model) {
    // Vertical Stack
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground);

    for i in 0..10 {
        let mut body = Body::new(Vec2::new(1.0, 1.0), 1.0);
        body.friction = 0.2;
        body.position = Vec2::new(random::<f32>() * 0.2 - 0.1, 0.51 + 1.05 * i as f32);
        model.world.add_body(body);
    }
}

fn demo5(model: &mut Model) {
    // Pyramid
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground);

    let mut x = Vec2::new(-6.0, 0.75);
    for i in 0..12 {
        let mut y = x;
        for _j in i..12 {
            let mut body = Body::new(Vec2::new(1.0, 1.0), 10.0);
            body.friction = 0.2;
            body.position = y;
            model.world.add_body(body);

            y.x += 1.125;
        }
        x.x += 0.5625;
        x.y += 2.0;
    }
}

fn demo6(model: &mut Model) {
    // A Teeter
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    model.world.add_body(body1);

    let mut body2 = Body::new(Vec2::new(12.0, 0.25), 10.0);
    body2.position = Vec2::new(0.0, 3.0);
    model.world.add_body(body2);

    let mut body3 = Body::new(Vec2::new(0.5, 0.5), 2.0);
    body3.position = Vec2::new(-5.0, 5.0);
    model.world.add_body(body3);

    let mut body4 = Body::new(Vec2::new(0.5, 0.5), 2.0);
    body4.position = Vec2::new(-5.5, 5.0);
    model.world.add_body(body4);

    let mut body5 = Body::new(Vec2::new(1.0, 1.0), 55.0);
    body5.position = Vec2::new(5.5, 15.0);
    model.world.add_body(body5);

    let joint = Joint::new(body1, body2, Vec2::new(0.0, 3.0), &model.world);
    model.world.add_joint(joint);
}

fn demo7(model: &mut Model) {
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground);

    let num_planks = 15;
    let mass = 10.0;
    let frequency_hz = 2.0;
    let damping_ratio = 0.7;
    let omega = 2.0 * std::f32::consts::PI * frequency_hz;
    let d = 2.0 * mass * damping_ratio * omega;
    let k = mass * omega * omega;
    let time_step = 1.0 / 60.0;
    let softness = 1.0 / (d + time_step * k);
    let bias_factor = time_step * k / (d + time_step * k);

    for i in 0..=num_planks {
        let mut plank = Body::new(Vec2::new(1.0, 0.25), mass);
        plank.friction = 0.2;
        plank.position = Vec2::new(-8.5 + 1.25 * i as f32, 5.0);
        model.world.add_body(plank);

        let mut joint = Joint::new(
            plank,
            ground,
            Vec2::new(-9.125 + 1.25 * i as f32, 5.0),
            &model.world,
        );
        joint.softness = softness;
        joint.bias_factor = bias_factor;
        model.world.add_joint(joint);
    }
}

// Dominos demo
fn demo8(model: &mut Model) {
    let mut b1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    b1.position = Vec2::new(0.0, -0.5 * b1.width.y);
    model.world.add_body(b1);

    let mut b = Body::new(Vec2::new(12.0, 0.5), f32::MAX);
    b.position = Vec2::new(-1.5, 10.0);
    model.world.add_body(b);

    for i in 0..10 {
        let mut domino = Body::new(Vec2::new(0.2, 2.0), 10.0);
        domino.position = Vec2::new(-6.0 + 1.0 * i as f32, 11.125);
        domino.friction = 0.1;
        model.world.add_body(domino);
    }

    let mut bb = Body::new(Vec2::new(14.0, 0.5), f32::MAX);
    bb.position = Vec2::new(1.0, 6.0);
    bb.rotation = 0.3;
    model.world.add_body(bb);

    let mut b2 = Body::new(Vec2::new(0.5, 3.0), f32::MAX);
    b2.position = Vec2::new(-7.0, 4.0);
    model.world.add_body(b2);

    let mut b3 = Body::new(Vec2::new(12.0, 0.25), 10.0);
    b3.position = Vec2::new(-0.9, 1.0);
    model.world.add_body(b3);

    let joint1 = Joint::new(b1, b3, Vec2::new(-2.0, 3.0), &model.world);
    model.world.add_joint(joint1);

    let mut b4 = Body::new(Vec2::new(0.5, 0.5), 16.0);
    b4.position = Vec2::new(-10.0, 15.0);
    b4.rotation = 0.0;
    b4.friction = 0.2;
    model.world.add_body(b4);

    let joint2 = Joint::new(b2, b4, Vec2::new(-7.0, 15.0), &model.world);
    model.world.add_joint(joint2);

    let mut b5 = Body::new(Vec2::new(2.0, 2.0), 10.0);
    b5.position = Vec2::new(6.0, 2.5);
    b5.friction = 0.1;
    model.world.add_body(b5);

    let joint3 = Joint::new(b1, b5, Vec2::new(6.0, 2.6), &model.world);
    model.world.add_joint(joint3);

    let mut b6 = Body::new(Vec2::new(2.0, 0.2), 10.0);
    b6.position = Vec2::new(6.0, 3.6);
    model.world.add_body(b6);

    let joint4 = Joint::new(b5, b6, Vec2::new(7.0, 3.5), &model.world);
    model.world.add_joint(joint4);
}

// Multi-pendulum demo
fn demo9(model: &mut Model) {
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    ground.rotation = 0.0;
    model.world.add_body(ground);

    let mut b1 = ground;
    let mass = 10.0;
    let frequency_hz = 4.0;
    let damping_ratio = 0.7;

    let omega = 2.0 * std::f32::consts::PI * frequency_hz;
    let d = 2.0 * mass * damping_ratio * omega;
    let k = mass * omega * omega;

    let time_step = model.time_step;
    let softness = 1.0 / (d + time_step * k);
    let bias_factor = time_step * k / (d + time_step * k);

    let y = 12.0;

    for i in 0..15 {
        let mut pendulum = Body::new(Vec2::new(0.75, 0.25), mass);
        pendulum.friction = 0.2;
        pendulum.position = Vec2::new(0.5 + i as f32, y);
        pendulum.rotation = 0.0;
        model.world.add_body(pendulum);

        let mut joint = Joint::new(b1, pendulum, Vec2::new(i as f32, y), &model.world);
        joint.softness = softness;
        joint.bias_factor = bias_factor;
        model.world.add_joint(joint);

        b1 = pendulum;
    }
}

fn update(_app: &App, _model: &mut Model, _update: Update) {
    if _model.is_first_frame {
        _model.world.step(_model.time_step);
        // Load the initial demo
        load_demo(_model);
        _model.is_first_frame = false;
    }
    _model.world.step(_model.time_step);
    if _model.load_demo_flag {
        load_demo(_model);
        _model.load_demo_flag = false;
    }

    if _model.bomb {
        launch_bomb(_model);
        _model.bomb = false;
    }

    let egui = &mut _model.egui;
    let settings = &mut _model.settings;

    egui.set_elapsed_time(_update.since_start);
    let ctx = egui.begin_frame();
    let demo_names = [
        "Demo 1: A Single Box",
        "Demo 2: Simple Pendulum",
        "Demo 3: Varying Friction Coefficients",
        "Demo 4: Randomized Stacking",
        "Demo 5: Pyramid Stacking",
        "Demo 6: A Teeter",
        "Demo 7: A Suspension Bridge",
        "Demo 8: Dominos",
        "Demo 9: Multi-pendulum",
    ];
    egui::Window::new("Settings").show(&ctx, |ui| {
        // Dropdown for selecting the demo
        ui.label("Select Demo:");
        egui::ComboBox::from_label("Demo Selection")
            .selected_text(format!("Demo {}", _model.demo_index + 1))
            .show_ui(ui, |ui| {
                for (i, name) in demo_names.iter().enumerate() {
                    ui.selectable_value(&mut _model.demo_index, i as u32, *name);
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

        if ui.button("launch bomb").clicked() {
            _model.bomb = true;
        }

        // Checkbox to enable a feature
        ui.checkbox(
            &mut _model.world.world_context.warm_starting,
            "Enable/Disable warm starting.",
        );
        ui.checkbox(
            &mut _model.world.world_context.position_correction,
            "Enable/Disable position correction.",
        );
        ui.checkbox(
            &mut _model.world.world_context.accumulate_impulse,
            "Enable/Disable accumulation of impulse.",
        );
    });
}

fn load_demo(model: &mut Model) {
    model.world.clear(); // Clear the current world bodies and joints

    match model.demo_index {
        0 => demo1(model),
        1 => demo2(model),
        2 => demo3(model),
        3 => demo4(model),
        4 => demo5(model),
        5 => demo6(model),
        6 => demo7(model),
        7 => demo8(model),
        8 => demo9(model),
        _ => {}
    }
}
fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    // Let egui handle things like keyboard and mouse input.
    model.egui.handle_raw_event(event);
}

fn key_pressed(_app: &App, model: &mut Model, key: Key) {
    match key {
        Key::Right => {
            model.world.step(model.time_step);
        }
        Key::Left => {
            model.world.step(-model.time_step);
        }
        Key::Return => {
            println!("Number of bodies {:?}", model.world.bodies.len());
            println!("World Bodies: {:?}", model.world.bodies);
            println!("{:?}", model.world.arbiters);
        }
        _other_key => {}
    }
}

fn view(app: &App, _model: &Model, frame: Frame) {
    let draw = app.draw();
    let draw = draw.scale(_model.settings.scale);
    let settings = &_model.settings;
    draw.background().color(SLATEGREY);
    for (num, body) in _model.world.iter_bodies().enumerate() {
        draw.rect()
            .x_y(body.position.x, body.position.y)
            .w_h(body.width.x, body.width.y)
            .rotate(body.rotation)
            .color(if num == 0 { DARKSEAGREEN } else { ORCHID });
    }

    for (_, arbiter) in _model.world.arbiters.iter() {
        for contact in arbiter.contacts.iter() {
            match contact {
                Some(contact) => {
                    draw.ellipse()
                        .x_y(contact.position.x, contact.position.y)
                        .radius(0.1)
                        .color(settings.color);
                    draw.arrow()
                        .start(pt2(contact.position.x, contact.position.y))
                        .end(pt2(
                            contact.position.x + contact.normal.x,
                            contact.position.y + contact.normal.y,
                        ))
                        .weight(0.05)
                        .color(LIGHTSALMON);
                }
                None => (),
            }
        }
    }
    for joint in _model.world.joints.iter() {
        let x1 = joint.body_1.borrow().position;
        let x2 = joint.body_2.borrow().position;
        let r1 = Mat2x2::new_from_angle(joint.body_1.borrow().rotation);
        let r2 = Mat2x2::new_from_angle(joint.body_2.borrow().rotation);
        let p1 = x1 + r1 * joint.local_anchor_1;
        let p2 = x2 + r2 * joint.local_anchor_2;
        draw.line()
            .start(pt2(x1.x, x1.y))
            .end(pt2(p1.x, p1.y))
            .weight(0.05)
            .color(SLATEBLUE);
        draw.line()
            .start(pt2(x2.x, x2.y))
            .end(pt2(p2.x, p2.y))
            .weight(0.05)
            .color(SLATEBLUE);
    }
    draw.to_frame(app, &frame).unwrap();
    _model.egui.draw_to_frame(&frame).unwrap();
}
