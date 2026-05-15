use std::collections::{HashMap, VecDeque};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use arc_swap::ArcSwap;
use flume::{bounded, Receiver, Sender};

use crate::can::FrameEntry;
use crate::comms_schema::Signal;
use crate::config::CanConfig;

#[derive(Clone, Debug, Default)]
pub struct GuiSnapshot {
    /// Latest frame per CAN ID
    pub frames: HashMap<u32, FrameEntry>,

    /// Rolling (timestamp_s, value) history indexed by Signal
    pub signal_history: Box<[VecDeque<(f64, f64)>; Signal::COUNT]>,
}

impl GuiSnapshot {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn apply_frame(
        &mut self,
        entry: &crate::can::FrameEntry,
        base_mc: u32,
        max_history: usize,
    ) {
        let ts = entry.received_at.timestamp_millis() as f64 / 1_000.0;
        for (sig, value) in
            crate::comms_schema::extract_signal_values(entry.id, &entry.data, base_mc)
        {
            let hist = &mut self.signal_history[sig as usize];
            hist.push_back((ts, value as f64));
            if hist.len() > max_history {
                hist.drain(..hist.len() - max_history);
            }
        }
        self.frames.insert(entry.id, entry.clone());
    }
}

/// Commands sent GUI → worker
#[derive(Debug, Clone)]
pub enum Command {
    SendFrame { id: u32, data: [u8; 8] },
    StartLog { path: PathBuf },
    StopLog,
    StopWorker,
}

/// Events sent worker → GUI
#[derive(Debug, Clone)]
pub enum Event {
    FrameReceived(FrameEntry),
    WorkerStatus(String),
    Error(String),
    Log(String),
}

#[derive(Clone)]
pub struct SharedData {
    snapshot: Arc<ArcSwap<GuiSnapshot>>,
    pub cmd_tx: Sender<Command>,
    cmd_rx: Receiver<Command>,
    pub event_tx: Sender<Event>,
    pub event_rx: Receiver<Event>,

    /// All CAN configuration in one place (interface, base addresses)
    pub can_config: Arc<Mutex<CanConfig>>,
}

impl SharedData {
    pub fn new() -> Self {
        let (cmd_tx, cmd_rx) = bounded(32);
        let (event_tx, event_rx) = bounded(1024);
        Self {
            snapshot: Arc::new(ArcSwap::from_pointee(GuiSnapshot::new())),
            cmd_tx,
            cmd_rx,
            event_tx,
            event_rx,
            can_config: Arc::new(Mutex::new(CanConfig::default())),
        }
    }

    pub fn get_can_config(&self) -> CanConfig {
        self.can_config
            .lock()
            .map(|g| g.clone())
            .unwrap_or_default()
    }

    pub fn set_can_config(&self, cfg: CanConfig) {
        if let Ok(mut g) = self.can_config.lock() {
            *g = cfg;
        }
    }

    /// Returns endpoints the worker thread owns (command rx, event tx, config)
    pub fn worker_endpoints(&self) -> (Receiver<Command>, Sender<Event>, Arc<Mutex<CanConfig>>) {
        (
            self.cmd_rx.clone(),
            self.event_tx.clone(),
            Arc::clone(&self.can_config),
        )
    }

    pub fn load_snapshot(&self) -> Arc<GuiSnapshot> {
        self.snapshot.load_full()
    }
    pub fn store_snapshot(&self, snap: GuiSnapshot) {
        self.snapshot.store(Arc::new(snap));
    }

    pub fn try_send_command(&self, cmd: Command) -> Result<(), flume::TrySendError<Command>> {
        self.cmd_tx.try_send(cmd)
    }
    pub fn try_recv_event(&self) -> Result<Event, flume::TryRecvError> {
        self.event_rx.try_recv()
    }
}
