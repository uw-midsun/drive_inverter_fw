//! Entry point for the drive inverter GUI
//!
//! - **Author:** Midnight Sun Team #24
//! - **Date:** 2026-05-18

mod can;
mod comms_schema;
mod config;
mod gui;
mod panels;
mod shared;

use eframe::NativeOptions;
use gui::GuiData;
use shared::SharedData;

fn main() {
    tracing_subscriber::fmt::init();

    let native_options = NativeOptions::default();
    let shared = SharedData::new();
    let _ = eframe::run_native(
        "Drive Inverter GUI",
        native_options,
        Box::new(|cc| Box::new(GuiData::new(cc, shared))),
    );
}
