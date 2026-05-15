use std::collections::HashSet;
use std::sync::Arc;

use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};

use crate::comms_schema::Signal;
use crate::shared::GuiSnapshot;

#[derive(Default)]
pub struct GraphState {
    pub selected: HashSet<Signal>,
}

pub fn show(ui: &mut egui::Ui, snapshot: &Arc<GuiSnapshot>, state: &mut GraphState) {
    ui.group(|ui| {
        ui.label("Graph");

        ui.horizontal_wrapped(|ui| {
            for &sig in &Signal::ALL {
                if snapshot.signal_history[sig as usize].is_empty() {
                    continue;
                }
                let mut checked = state.selected.contains(&sig);
                if ui.checkbox(&mut checked, sig.name()).changed() {
                    if checked {
                        state.selected.insert(sig);
                    } else {
                        state.selected.remove(&sig);
                    }
                }
            }
        });

        let plot_height = 200.0_f32.max(ui.available_height() * 0.4);

        Plot::new("signal_plot")
            .height(plot_height)
            .x_axis_label("Time (s)")
            .show(ui, |plot_ui| {
                for &sig in state.selected.iter() {
                    let history = &snapshot.signal_history[sig as usize];
                    if !history.is_empty() {
                        let points: PlotPoints = history.iter().map(|&(t, v)| [t, v]).collect();
                        plot_ui.line(Line::new(points).name(sig.name()));
                    }
                }
            });
    });
}
