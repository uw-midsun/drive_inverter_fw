use std::time::{Duration, Instant};

use eframe::egui;

use crate::comms_schema;
use crate::shared::{Command, SharedData};

pub struct ControlState {
    pub velocity_rpm: f32,
    pub current_limit: f32,
    pub heartbeat: Option<Instant>,
}

impl Default for ControlState {
    fn default() -> Self {
        Self {
            velocity_rpm: 0.0,
            current_limit: 0.5,
            heartbeat: None,
        }
    }
}

impl ControlState {
    pub fn tick(&mut self, shared: &SharedData, base_dc: u32) {
        if let Some(last_sent) = self.heartbeat {
            if last_sent.elapsed() >= Duration::from_millis(100) {
                let data = comms_schema::encode_drive(self.velocity_rpm, self.current_limit);
                let _ = shared.try_send_command(Command::SendFrame {
                    id: base_dc + comms_schema::DC_DRIVE,
                    data,
                });
                self.heartbeat = Some(Instant::now());
            }
        }
    }
}

pub fn show(ui: &mut egui::Ui, shared: &SharedData, state: &mut ControlState, base_dc: u32) {
    let drive_id = base_dc + comms_schema::DC_DRIVE;

    ui.heading("Motor Control");
    ui.separator();

    ui.add(
        egui::Slider::new(&mut state.velocity_rpm, -6000.0..=6000.0)
            .text("Velocity (rpm)")
            .step_by(10.0),
    );
    ui.add(
        egui::Slider::new(&mut state.current_limit, 0.0..=1.0)
            .text("Current limit (pu)")
            .step_by(0.01),
    );

    ui.separator();
    ui.horizontal(|ui| {
        if ui.button("Send Drive Command").clicked() {
            let data = comms_schema::encode_drive(state.velocity_rpm, state.current_limit);
            let _ = shared.try_send_command(Command::SendFrame { id: drive_id, data });
        }
        if ui.button("Disable (zero current)").clicked() {
            let data = comms_schema::encode_drive(0.0, 0.0);
            let _ = shared.try_send_command(Command::SendFrame { id: drive_id, data });
            state.current_limit = 0.0;
            state.heartbeat = None;
        }
    });

    ui.separator();

    let mut active = state.heartbeat.is_some();
    if ui.checkbox(&mut active, "Auto-send heartbeat (100 ms)").changed() {
        state.heartbeat = if active {
            Some(Instant::now() - Duration::from_millis(101))
        } else {
            None
        };
    }

    ui.separator();
    ui.label(
        egui::RichText::new(
            "Drive commands must be sent periodically (<250 ms) or the controller will time out.",
        )
        .weak()
        .small(),
    );
}
