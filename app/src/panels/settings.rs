use eframe::egui;

use crate::config::CanConfig;
use crate::shared::SharedData;

pub struct SettingsState {
    pub iface: String,
    pub base_mc: String,
    pub base_dc: String,
    error: Option<String>,
}

impl SettingsState {
    pub fn new(iface: String, base_mc: u32, base_dc: u32) -> Self {
        Self {
            iface,
            base_mc: format!("0x{:X}", base_mc),
            base_dc: format!("0x{:X}", base_dc),
            error: None,
        }
    }
}

pub fn show(ctx: &egui::Context, shared: &SharedData, state: &mut SettingsState, open: &mut bool) {
    if !*open {
        return;
    }

    egui::Window::new("Settings")
        .open(open)
        .resizable(false)
        .show(ctx, |ui| {
            egui::Grid::new("settings_grid")
                .num_columns(2)
                .spacing([12.0, 6.0])
                .show(ui, |ui| {
                    ui.label("CAN Interface");
                    ui.add(
                        egui::TextEdit::singleline(&mut state.iface)
                            .desired_width(120.0)
                            .hint_text("can0, vcan0, slcan0 …"),
                    );
                    ui.end_row();

                    ui.label("MC Base (hex)");
                    ui.add(
                        egui::TextEdit::singleline(&mut state.base_mc)
                            .desired_width(120.0)
                            .hint_text("0x400"),
                    );
                    ui.end_row();

                    ui.label("DC Base (hex)");
                    ui.add(
                        egui::TextEdit::singleline(&mut state.base_dc)
                            .desired_width(120.0)
                            .hint_text("0x500"),
                    );
                    ui.end_row();
                });

            if let Some(err) = &state.error {
                ui.colored_label(egui::Color32::RED, err);
            }

            ui.separator();
            if ui.button("Apply").clicked() {
                match CanConfig::parse(&state.iface, &state.base_mc, &state.base_dc) {
                    Ok(cfg) => {
                        state.error = None;
                        shared.set_can_config(cfg);
                    }
                    Err(e) => state.error = Some(e),
                }
            }

            ui.add_space(2.0);
            ui.label(
                egui::RichText::new(
                    "Base address changes apply to new frames immediately.\n\
                     Interface change takes effect on the next worker start.",
                )
                .weak()
                .small(),
            );
        });
}
