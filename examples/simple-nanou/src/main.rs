use nannou::prelude::*;
use nannou_egui::{self, egui, Egui};
use sylt_2d::body::Body;
use sylt_2d::joint::Joint;
use sylt_2d::math_utils::{Mat2x2, Vec2};
use sylt_2d::world::World;

fn main() {
    nannou::app(model).update(update).run();
}

struct EguiSettings {
    resolution: u32,
    scale: f32,
    rotation: f32,
    color: Srgb<u8>,
    position: Vec2,
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
}

fn model(app: &App) -> Model {
    let _window = app
        .new_window()
        .view(view)
        .raw_event(raw_window_event)
        .build()
        .unwrap();
    let window = app.window(_window).unwrap();
    let egui = Egui::from_window(&window);
    Model {
        _window,
        world: World::new(Vec2::new(0.0, -10.0), 10),
        demo_index: 0,
        bomb: None,
        time_step: 1.0 / 60.0,
        egui,
        settings: EguiSettings {
            resolution: 10,
            scale: 5.0,
            rotation: 0.0,
            color: WHITE,
            position: Vec2::new(0.0, 0.0),
        },
        is_first_frame: true,
    }
}

fn demo_1(_model: &mut Model) {
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    _model.world.add_body(body1);

    let mut body2 = Body::new(Vec2::new(1.0, 1.0), 200.0);
    body2.position = Vec2::new(0.0, 4.0);
    _model.world.add_body(body2);
}
fn update(_app: &App, _model: &mut Model, _update: Update) {
    _model.world.step(_model.time_step);
    if _model.is_first_frame {
        demo_1(_model);
        _model.is_first_frame = false;
    }
    let egui = &mut _model.egui;
    let settings = &mut _model.settings;

    egui.set_elapsed_time(_update.since_start);
    let ctx = egui.begin_frame();

    egui::Window::new("Settings").show(&ctx, |ui| {
        // Resolution slider
        ui.label("Resolution:");
        ui.add(egui::Slider::new(&mut settings.resolution, 1..=40));

        // Scale slider
        ui.label("Scale:");
        ui.add(egui::Slider::new(&mut settings.scale, 0.0..=1000.0));

        // Rotation slider
        ui.label("Rotation:");
        ui.add(egui::Slider::new(&mut settings.rotation, 0.0..=360.0));

        // Random color button
        let clicked = ui.button("Random color").clicked();

        if clicked {
            settings.color = rgb(random(), random(), random());
        }
    });
}

fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    // Let egui handle things like keyboard and mouse input.
    model.egui.handle_raw_event(event);
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
            .z_degrees(body.rotation)
            .color(if num == 0 { DARKSEAGREEN } else { ORCHID });
    }

    for (_, arbiter) in _model.world.arbiters.iter() {
        for contact in arbiter.contacts.iter() {
            draw.ellipse()
                .x_y(contact.position.x, contact.position.y)
                .radius(0.1)
                .color(settings.color);
        }
    }
    frame.clear(SLATEGREY);
    draw.to_frame(app, &frame).unwrap();
    _model.egui.draw_to_frame(&frame).unwrap();
}
