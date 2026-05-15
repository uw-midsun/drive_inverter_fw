use std::time::Duration;

use eframe::egui;

use crate::can::run as can_run;
use crate::comms_schema;
use crate::config::{parse_hex, CanConfig};
use crate::panels;
use crate::shared::{Command, Event, SharedData};

const STORAGE_KEY_DARK: &str = "ui.dark_mode";
const STORAGE_KEY_BASE_MC: &str = "can.base_mc";
const STORAGE_KEY_BASE_DC: &str = "can.base_dc";
const STORAGE_KEY_IFACE: &str = "can.iface";
const MAX_HISTORY: usize = 2_000;

pub struct GuiData {
    shared: SharedData,

    show_control: bool,
    show_settings: bool,

    worker_started: bool,
    worker_handle: Option<std::thread::JoinHandle<()>>,
    last_worker_status: Option<String>,

    is_dark: bool,

    log: panels::log::LogState,
    graph: panels::graph::GraphState,
    control: panels::control::ControlState,
    settings: panels::settings::SettingsState,
}

impl GuiData {
    pub fn new(cc: &eframe::CreationContext<'_>, shared: SharedData) -> Self {
        let is_dark = load_bool(cc.storage, STORAGE_KEY_DARK, false);
        let iface = cc
            .storage
            .and_then(|s| s.get_string(STORAGE_KEY_IFACE))
            .unwrap_or_else(|| "can0".to_string());
        let base_mc = load_hex(cc.storage, STORAGE_KEY_BASE_MC, comms_schema::BASE_MC);
        let base_dc = load_hex(cc.storage, STORAGE_KEY_BASE_DC, comms_schema::BASE_DC);

        shared.set_can_config(CanConfig {
            iface: iface.clone(),
            base_mc,
            base_dc,
        });
        cc.egui_ctx.set_visuals(make_visuals(is_dark));

        Self {
            shared,
            show_control: false,
            show_settings: false,
            worker_started: false,
            worker_handle: None,
            last_worker_status: None,
            is_dark,
            log: panels::log::LogState::default(),
            graph: panels::graph::GraphState::default(),
            control: panels::control::ControlState::default(),
            settings: panels::settings::SettingsState::new(iface, base_mc, base_dc),
        }
    }

    fn drain_events(&mut self) {
        let cfg = self.shared.get_can_config();
        let mut new_frames: Vec<crate::can::FrameEntry> = Vec::new();

        while let Ok(ev) = self.shared.try_recv_event() {
            match ev {
                Event::WorkerStatus(s) => self.last_worker_status = Some(s),
                Event::FrameReceived(e) => new_frames.push(e),
                Event::Error(e) => tracing::warn!("worker: {e}"),
                Event::Log(s) => tracing::info!("log: {s}"),
            }
        }

        if new_frames.is_empty() {
            return;
        }

        let mut snap = (*self.shared.load_snapshot()).clone();
        for entry in &new_frames {
            snap.apply_frame(entry, cfg.base_mc, MAX_HISTORY);
        }
        self.shared.store_snapshot(snap);
    }

    fn top_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.strong("Drive Inverter");
                ui.separator();

                let theme_label = if self.is_dark { "Light" } else { "Dark" };
                if ui.button(theme_label).clicked() {
                    self.is_dark = !self.is_dark;
                    ctx.set_visuals(make_visuals(self.is_dark));
                }

                let is_fs = ctx.input(|i| i.viewport().fullscreen.unwrap_or(false));
                if ui
                    .button(if is_fs { "Window" } else { "Fullscreen" })
                    .clicked()
                {
                    ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(!is_fs));
                }

                ui.separator();
                let cfg = self.shared.get_can_config();
                ui.label(format!("CAN: {}", cfg.iface));

                if self.worker_started {
                    if ui.button("■ Stop").clicked() {
                        let _ = self.shared.try_send_command(Command::StopWorker);
                        self.worker_started = false;
                        self.control.heartbeat = None;
                    }
                    ui.add_enabled(false, egui::Button::new("● Running"));
                } else if ui.button("▶ Start").clicked() {
                    let prev_finished = self
                        .worker_handle
                        .as_ref()
                        .map(|h| h.is_finished())
                        .unwrap_or(true);

                    if prev_finished {
                        self.worker_handle = None;
                        self.worker_started = true;
                        let (cmd_rx, event_tx, config) = self.shared.worker_endpoints();
                        self.worker_handle = Some(std::thread::spawn(move || {
                            can_run(cmd_rx, event_tx, config)
                        }));
                    } else {
                        self.last_worker_status =
                            Some("Previous worker still stopping, trying again".to_string());
                    }
                }

                ui.separator();
                if ui.button("Settings").clicked() {
                    self.show_settings = true;
                }
                if ui.button("Control").clicked() {
                    self.show_control = true;
                }

                if let Some(status) = &self.last_worker_status {
                    ui.separator();
                    ui.label(status.as_str());
                }
            });
        });
    }
}

impl eframe::App for GuiData {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        storage.set_string(STORAGE_KEY_DARK, self.is_dark.to_string());
        let cfg = self.shared.get_can_config();
        storage.set_string(STORAGE_KEY_IFACE, cfg.iface);
        storage.set_string(STORAGE_KEY_BASE_MC, format!("0x{:X}", cfg.base_mc));
        storage.set_string(STORAGE_KEY_BASE_DC, format!("0x{:X}", cfg.base_dc));
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        let _ = self.shared.try_send_command(Command::StopWorker);
        if let Some(handle) = self.worker_handle.take() {
            let _ = handle.join();
        }
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.drain_events();
        self.top_bar(ctx);
        panels::settings::show(
            ctx,
            &self.shared,
            &mut self.settings,
            &mut self.show_settings,
        );

        let cfg = self.shared.get_can_config();

        if self.worker_started {
            self.control.tick(&self.shared, cfg.base_dc);
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            let snapshot = self.shared.load_snapshot();

            ui.horizontal(|ui| {
                panels::graph::show(ui, &snapshot, &mut self.graph);
                panels::log::show(ui, &self.shared, &mut self.log);
            });

            ui.separator();
            panels::telemetry::show(ui, &snapshot, cfg.base_mc);
        });

        if self.show_control {
            egui::Window::new("Control")
                .open(&mut self.show_control)
                .show(ctx, |ui| {
                    panels::control::show(ui, &self.shared, &mut self.control, cfg.base_dc);
                });
        }

        ctx.request_repaint_after(Duration::from_millis(if self.worker_started {
            16
        } else {
            200
        }));
    }
}

fn load_hex(storage: Option<&dyn eframe::Storage>, key: &str, default: u32) -> u32 {
    storage
        .and_then(|s| s.get_string(key))
        .and_then(|v| parse_hex(&v))
        .unwrap_or(default)
}

fn load_bool(storage: Option<&dyn eframe::Storage>, key: &str, default: bool) -> bool {
    storage
        .and_then(|s| s.get_string(key))
        .and_then(|v| v.parse::<bool>().ok())
        .unwrap_or(default)
}

fn make_visuals(dark: bool) -> egui::Visuals {
    if dark {
        egui::Visuals::dark()
    } else {
        egui::Visuals::light()
    }
}
