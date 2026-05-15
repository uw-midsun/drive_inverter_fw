use std::fmt;

pub const BASE_MC: u32 = 0x400; // default motor controller base
pub const BASE_DC: u32 = 0x500; // default driver controls base

// == MC frame offsets (from base_mc) ========================================
pub const MC_BUS: u32 = 0x02;
pub const MC_VELOCITY: u32 = 0x03;
pub const MC_PHASE_CURRENT: u32 = 0x04;
pub const MC_MOTOR_VOLTAGE: u32 = 0x05;
pub const MC_MOTOR_CURRENT: u32 = 0x06;
pub const MC_MOTOR_BEMF: u32 = 0x07;
pub const MC_RAIL_15V_1V65: u32 = 0x08;
pub const MC_RAIL_2V5_1V2: u32 = 0x09;
pub const MC_FAN_SPEED: u32 = 0x0A;
pub const MC_TEMP_SINK_MOTOR: u32 = 0x0B;
pub const MC_TEMP_DSP: u32 = 0x0C;
pub const MC_ODOMETER: u32 = 0x0E;

// == DC frame offsets (from base_dc) ========================================
pub const DC_DRIVE: u32 = 0x01;

const MC_RANGE: u32 = 0x10;

// == Signals =================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(usize)]
pub enum Signal {
    BusVoltage = 0,
    BusCurrent,
    VelocityRpm,
    VelocityMs,
    PhaseIb,
    PhaseIc,
    MotorVd,
    MotorVq,
    MotorId,
    MotorIq,
    BemfVd,
    BemfVq,
    TempHeatsink,
    TempMotor,
    TempDsp,
    FanRpm,
    OdometerM,
    AmpHours,
}

impl Signal {
    pub const COUNT: usize = 18;
    pub const ALL: [Signal; Self::COUNT] = [
        Signal::BusVoltage,
        Signal::BusCurrent,
        Signal::VelocityRpm,
        Signal::VelocityMs,
        Signal::PhaseIb,
        Signal::PhaseIc,
        Signal::MotorVd,
        Signal::MotorVq,
        Signal::MotorId,
        Signal::MotorIq,
        Signal::BemfVd,
        Signal::BemfVq,
        Signal::TempHeatsink,
        Signal::TempMotor,
        Signal::TempDsp,
        Signal::FanRpm,
        Signal::OdometerM,
        Signal::AmpHours,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Signal::BusVoltage => "bus_voltage",
            Signal::BusCurrent => "bus_current",
            Signal::VelocityRpm => "velocity_rpm",
            Signal::VelocityMs => "velocity_ms",
            Signal::PhaseIb => "phase_ib",
            Signal::PhaseIc => "phase_ic",
            Signal::MotorVd => "motor_vd",
            Signal::MotorVq => "motor_vq",
            Signal::MotorId => "motor_id",
            Signal::MotorIq => "motor_iq",
            Signal::BemfVd => "bemf_vd",
            Signal::BemfVq => "bemf_vq",
            Signal::TempHeatsink => "temp_heatsink",
            Signal::TempMotor => "temp_motor",
            Signal::TempDsp => "temp_dsp",
            Signal::FanRpm => "fan_rpm",
            Signal::OdometerM => "odometer_m",
            Signal::AmpHours => "amp_hours",
        }
    }
}

impl fmt::Display for Signal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

// == Decode ==================================================================

pub(crate) fn f32_pair(data: &[u8; 8]) -> (f32, f32) {
    let lo = f32::from_le_bytes(data[0..4].try_into().unwrap());
    let hi = f32::from_le_bytes(data[4..8].try_into().unwrap());
    (lo, hi)
}

pub fn extract_signal_values(id: u32, data: &[u8; 8], base_mc: u32) -> Vec<(Signal, f32)> {
    if id < base_mc || id >= base_mc + MC_RANGE {
        return vec![];
    }
    let (lo, hi) = f32_pair(data);
    match id - base_mc {
        MC_BUS => vec![(Signal::BusVoltage, lo), (Signal::BusCurrent, hi)],
        MC_VELOCITY => vec![(Signal::VelocityRpm, lo), (Signal::VelocityMs, hi)],
        MC_PHASE_CURRENT => vec![(Signal::PhaseIb, lo), (Signal::PhaseIc, hi)],
        MC_MOTOR_VOLTAGE => vec![(Signal::MotorVd, lo), (Signal::MotorVq, hi)],
        MC_MOTOR_CURRENT => vec![(Signal::MotorId, lo), (Signal::MotorIq, hi)],
        MC_MOTOR_BEMF => vec![(Signal::BemfVd, lo), (Signal::BemfVq, hi)],
        MC_TEMP_SINK_MOTOR => vec![(Signal::TempHeatsink, lo), (Signal::TempMotor, hi)],
        MC_TEMP_DSP => vec![(Signal::TempDsp, lo)],
        MC_FAN_SPEED => vec![(Signal::FanRpm, lo)],
        MC_ODOMETER => vec![(Signal::OdometerM, lo), (Signal::AmpHours, hi)],
        _ => vec![],
    }
}

// == Encode ==================================================================

pub fn encode_drive(velocity_rpm: f32, current_limit: f32) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0..4].copy_from_slice(&velocity_rpm.to_le_bytes());
    out[4..8].copy_from_slice(&current_limit.to_le_bytes());
    out
}
