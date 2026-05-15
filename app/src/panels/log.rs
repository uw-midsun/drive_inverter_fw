use std::path::PathBuf;

use eframe::egui;

use crate::shared::{Command, SharedData};

pub struct LogState {
    pub path: String,
    pub active: bool,
}

impl Default for LogState {
    fn default() -> Self {
        Self { path: String::new(), active: false }
    }
}

pub fn show(ui: &mut egui::Ui, shared: &SharedData, state: &mut LogState) {
    ui.group(|ui| {
        ui.label("CAN Log");

        ui.add(
            egui::TextEdit::singleline(&mut state.path)
                .desired_width(160.0)
                .hint_text("output.log")
                .interactive(!state.active),
        );

        ui.horizontal(|ui| {
            if !state.active {
                let can_start = !state.path.trim().is_empty();
                ui.add_enabled_ui(can_start, |ui| {
                    if ui.button("▶ Start").clicked() {
                        let path = PathBuf::from(state.path.trim());
                        let _ = shared.try_send_command(Command::StartLog { path });
                        state.active = true;
                    }
                });
            } else {
                if ui.button("■ Stop").clicked() {
                    let _ = shared.try_send_command(Command::StopLog);
                    state.active = false;
                }
                ui.label(egui::RichText::new("● Recording").color(egui::Color32::RED));
            }
        });
    });
}
