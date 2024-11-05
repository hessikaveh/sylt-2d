use nannou::prelude::*;
use nannou_egui::{self, egui, Egui};
use sylt_2d::arbiter::{Contact, ContactInfo};
use sylt_2d::body::Body;
use sylt_2d::joint::Joint;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;
fn main() {
    nannou::app(model).update(update).run();
}

struct EguiSettings {
    scale: f32,
    color: Srgb<u8>,
}

struct Model {
    _window: window::Id,
    time_step: f32,
    demo_index: u32,
    world: World,
    bomb: Option<Body>,
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
    let mut world = World::new(Vec2::new(0.0, -10.0), 10);
    //world.world_context.warm_starting = false;
    //world.world_context.accumulate_impulse = false;
    //world.world_context.position_correction = false;
    Model {
        _window,
        world,
        demo_index: 0,
        bomb: None,
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

fn demo1(_model: &mut Model) {
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    _model.world.add_body(body1);

    let mut body2 = Body::new(Vec2::new(1.0, 1.0), 200.0);
    body2.position = Vec2::new(0.0, 4.0);
    _model.world.add_body(body2);
}

fn demo2(model: &mut Model) {
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

    let joint = Joint::new(body1, body2, Vec2::new(0.0, 11.0));
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
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    model.world.add_body(body1);

    let mut body2 = Body::new(Vec2::new(12.0, 0.25), 100.0);
    body2.position = Vec2::new(0.0, 1.0);
    model.world.add_body(body2);

    let mut joint = Joint::new(body1, body2, Vec2::new(0.0, 1.0));
    model.world.add_joint(joint);
}

fn demo7(model: &mut Model) {
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground);

    let num_planks = 15;
    let mass = 50.0;
    let frequency_hz = 2.0;
    let damping_ratio = 0.7;
    let omega = 2.0 * std::f32::consts::PI * frequency_hz;
    let d = 2.0 * mass * damping_ratio * omega;
    let k = mass * omega * omega;
    let time_step = 1.0 / 60.0;
    let softness = 1.0 / (d + time_step * k);
    let bias_factor = time_step * k / (d + time_step * k);

    for i in 0..num_planks {
        let mut plank = Body::new(Vec2::new(1.0, 0.25), mass);
        plank.friction = 0.2;
        plank.position = Vec2::new(-8.5 + 1.25 * i as f32, 5.0);
        model.world.add_body(plank);

        let mut joint = Joint::new(plank, ground, Vec2::new(-9.125 + 1.25 * i as f32, 5.0));
        joint.softness = softness;
        joint.bias_factor = bias_factor;
        model.world.add_joint(joint);
    }
}
fn update(_app: &App, _model: &mut Model, _update: Update) {
    if _model.is_first_frame {
        _model.world.step(_model.time_step);
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
    model.world.clear(); // Clear the current world bodies and joints

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
        Key::Right => {
            model.world.step(model.time_step);
        }
        Key::Left => {
            model.world.step(-model.time_step);
        }
        Key::Return => {
            println!(
                "{:?}",
                model.world.bodies.get(0).unwrap().id < model.world.bodies.get(1).unwrap().id
            );
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
    for (num, body) in _model.world.bodies.iter().enumerate() {
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
    draw.to_frame(app, &frame).unwrap();
    _model.egui.draw_to_frame(&frame).unwrap();
}
