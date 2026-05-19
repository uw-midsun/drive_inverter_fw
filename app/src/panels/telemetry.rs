//! Live telemetry readout grid panel
//!
//! - **Author:** Midnight Sun Team #24
//! - **Date:** 2026-05-18

use std::collections::HashMap;
use std::sync::Arc;

use eframe::egui;

use crate::can::FrameEntry;
use crate::comms_schema;
use crate::shared::GuiSnapshot;

pub fn show(ui: &mut egui::Ui, snapshot: &Arc<GuiSnapshot>, base_mc: u32) {
    let frames = &snapshot.frames;

    let (bus_v, bus_i) = pair(frames, base_mc + comms_schema::MC_BUS);
    let (rpm, vel_ms) = pair(frames, base_mc + comms_schema::MC_VELOCITY);
    let (hs_temp, mt_temp) = pair(frames, base_mc + comms_schema::MC_TEMP_SINK_MOTOR);
    let (dsp_tmp, _) = pair(frames, base_mc + comms_schema::MC_TEMP_DSP);
    let (r15v, r165v) = pair(frames, base_mc + comms_schema::MC_RAIL_15V_1V65);
    let (r25v, r12v) = pair(frames, base_mc + comms_schema::MC_RAIL_2V5_1V2);
    let (ib, ic) = pair(frames, base_mc + comms_schema::MC_PHASE_CURRENT);
    let (bemf_d, bemf_q) = pair(frames, base_mc + comms_schema::MC_MOTOR_BEMF);
    let (vd, vq) = pair(frames, base_mc + comms_schema::MC_MOTOR_VOLTAGE);
    let (id, iq) = pair(frames, base_mc + comms_schema::MC_MOTOR_CURRENT);
    let bus_power = bus_v.zip(bus_i).map(|(v, i)| v * i);

    // == Row 1 ================================================================
    ui.horizontal(|ui| {
        group(ui, "Power", |ui| {
            row(ui, "Bus Voltage", bus_v, "V", 2);
            row(ui, "Bus Current", bus_i, "A", 2);
            row(ui, "Bus Power", bus_power, "W", 1);
        });
        group(ui, "Velocity", |ui| {
            row(ui, "Speed", rpm, "rpm", 1);
            row(ui, "Speed", vel_ms, "m/s", 2);
        });
        group(ui, "Temperatures", |ui| {
            row(ui, "Motor", mt_temp, "°C", 1);
            row(ui, "Heatsink", hs_temp, "°C", 1);
            row(ui, "DSP", dsp_tmp, "°C", 1);
        });
        group(ui, "Voltages", |ui| {
            row(ui, "15V Rail", r15v, "V", 2);
            row(ui, "1.65V Rail", r165v, "V", 3);
            row(ui, "2.5V Rail", r25v, "V", 3);
            row(ui, "1.2V Rail", r12v, "V", 3);
        });
    });

    ui.add_space(4.0);

    // == Row 2 ================================================================
    ui.horizontal(|ui| {
        group(ui, "Phase Current", |ui| {
            row(ui, "Ib", ib, "A", 2);
            row(ui, "Ic", ic, "A", 2);
        });
        group(ui, "Motor BEMF", |ui| {
            row(ui, "Vd", bemf_d, "V", 2);
            row(ui, "Vq", bemf_q, "V", 2);
        });
        group(ui, "Motor Voltage", |ui| {
            row(ui, "Vd", vd, "V", 2);
            row(ui, "Vq", vq, "V", 2);
        });
        group(ui, "Motor Current", |ui| {
            row(ui, "Id", id, "A", 2);
            row(ui, "Iq", iq, "A", 2);
        });
    });
}

// == Helpers ==================================================================

fn pair(frames: &HashMap<u32, FrameEntry>, id: u32) -> (Option<f32>, Option<f32>) {
    frames
        .get(&id)
        .map(|e| {
            let (lo, hi) = comms_schema::f32_pair(&e.data);
            (Some(lo), Some(hi))
        })
        .unwrap_or((None, None))
}

fn group(ui: &mut egui::Ui, title: &'static str, add_contents: impl FnOnce(&mut egui::Ui)) {
    ui.group(|ui| {
        ui.set_min_width(150.0);
        ui.strong(title);
        ui.separator();
        egui::Grid::new(title)
            .num_columns(2)
            .spacing([12.0, 2.0])
            .show(ui, add_contents);
    });
}

fn row(ui: &mut egui::Ui, label: &str, val: Option<f32>, unit: &str, decimals: usize) {
    ui.label(label);
    match val {
        Some(v) => {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.monospace(format!("{:.prec$} {}", v, unit, prec = decimals));
            });
        }
        None => {
            ui.weak("---");
        }
    }
    ui.end_row();
}
