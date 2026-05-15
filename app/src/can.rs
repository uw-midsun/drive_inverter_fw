use chrono::{DateTime, Utc};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use flume::{Receiver, Sender};
use socketcan::{CanDataFrame, CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Socket, StandardId};

use crate::config::CanConfig;
use crate::shared::{Command, Event};

#[derive(Clone, Debug)]
pub struct FrameEntry {
    pub id: u32,
    pub data: [u8; 8],
    pub received_at: DateTime<Utc>,
}

// == Worker state machine ====================================================

enum WorkerState {
    Connecting { retry_after: Instant },
    Running { socket: CanSocket },
}

impl WorkerState {
    fn socket_mut(&mut self) -> Option<&mut CanSocket> {
        match self {
            WorkerState::Running { socket } => Some(socket),
            WorkerState::Connecting { .. } => None,
        }
    }
}

// == CAN worker ==============================================================

pub fn run(cmd_rx: Receiver<Command>, event_tx: Sender<Event>, config: Arc<Mutex<CanConfig>>) {
    use std::io::ErrorKind;

    let mut state = WorkerState::Connecting { retry_after: Instant::now() };
    let mut log_file: Option<BufWriter<File>> = None;

    loop {
        // Commands are always drained here
        // StopWorker has one exit point
        while let Ok(cmd) = cmd_rx.try_recv() {
            if let Command::StopWorker = cmd {
                return;
            }
            handle_command(cmd, state.socket_mut(), &event_tx, &mut log_file);
        }

        match &mut state {
            WorkerState::Connecting { retry_after } => {
                let remaining = retry_after.saturating_duration_since(Instant::now());
                if !remaining.is_zero() {

                    // Block until retry time or a command arrives
                    match cmd_rx.recv_timeout(remaining) {
                        Ok(Command::StopWorker) => return,
                        Ok(cmd) => handle_command(cmd, None, &event_tx, &mut log_file),
                        Err(_) => {}
                    }
                    continue;
                }

                let iface = config
                    .lock()
                    .map(|g| g.iface.clone())
                    .unwrap_or_else(|_| "can0".to_string());

                match CanSocket::open(&iface) {
                    Ok(socket) => {
                        let _ = socket.set_read_timeout(Some(Duration::from_millis(200)));
                        let _ = event_tx.try_send(Event::WorkerStatus(format!("Connected to {iface}")));
                        state = WorkerState::Running { socket };
                    }
                    Err(e) => {
                        let _ = event_tx.try_send(Event::WorkerStatus(format!("Cannot open {iface}: {e}")));
                        *retry_after = Instant::now() + Duration::from_secs(2);
                    }
                }
            }

            WorkerState::Running { socket } => {
                match socket.read_frame() {
                    Ok(frame) => {
                        let raw_id: u32 = match frame.id() {
                            socketcan::Id::Standard(sid) => sid.as_raw() as u32,
                            socketcan::Id::Extended(eid) => eid.as_raw(),
                        };
                        let bytes = frame.data();
                        let len = bytes.len().min(8);
                        let mut data = [0u8; 8];
                        data[..len].copy_from_slice(&bytes[..len]);
                        let received_at = Utc::now();

                        if let Some(ref mut w) = log_file {
                            let ts = received_at.timestamp_millis() as f64 / 1_000.0;
                            write!(w, "{:.3},{:#010X}", ts, raw_id).ok();
                            for b in &data {
                                write!(w, " {:02X}", b).ok();
                            }
                            writeln!(w).ok();
                        }

                        let _ = event_tx.try_send(Event::FrameReceived(FrameEntry { id: raw_id, data, received_at }));
                    }
                    Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                    Err(e) => {
                        let _ = event_tx.try_send(Event::Error(format!("CAN read error: {e}")));
                        state = WorkerState::Connecting {
                            retry_after: Instant::now() + Duration::from_secs(2),
                        };
                    }
                }
            }
        }
    }
}

// == Command handler =========================================================

fn handle_command(
    cmd: Command,
    socket: Option<&mut CanSocket>,
    event_tx: &Sender<Event>,
    log_file: &mut Option<BufWriter<File>>,
) {
    match cmd {
        Command::SendFrame { id, data } => {
            let Some(socket) = socket else {
                let _ = event_tx.try_send(Event::Error("Frame dropped, CAN not connected".into()));
                return;
            };

            let can_id = if id <= 0x7FF {
                match StandardId::new(id as u16) {
                    Some(sid) => socketcan::Id::Standard(sid),
                    None => {
                        let _ = event_tx.try_send(Event::Error(format!("Invalid standard CAN ID: {:#X}", id)));
                        return;
                    }
                }
            } else {
                match ExtendedId::new(id) {
                    Some(eid) => socketcan::Id::Extended(eid),
                    None => {
                        let _ = event_tx.try_send(Event::Error(format!("Invalid extended CAN ID: {:#X}", id)));
                        return;
                    }
                }
            };

            match CanDataFrame::new(can_id, &data) {
                Some(frame) => {
                    if let Err(e) = socket.write_frame(&CanFrame::Data(frame)) {
                        let _ = event_tx.try_send(Event::Error(format!("Send failed: {e}")));
                    }
                }
                None => {
                    let _ = event_tx.try_send(Event::Error("Invalid CAN frame".into()));
                }
            }
        }

        Command::StartLog { path } => {
            match File::create(&path)
                .map(BufWriter::new)
                .and_then(|mut w| {
                    writeln!(w, "timestamp_s,can_id,data")?;
                    Ok(w)
                }) {
                Ok(w) => {
                    *log_file = Some(w);
                    let _ = event_tx.try_send(Event::Log(format!("Logging to {}", path.display())));
                }
                Err(e) => {
                    let _ = event_tx.try_send(Event::Error(format!("Log open failed: {e}")));
                }
            }
        }

        Command::StopLog => {
            if let Some(mut w) = log_file.take() {
                let _ = w.flush();
            }
            let _ = event_tx.try_send(Event::Log("Logging stopped".into()));
        }

        Command::StopWorker => unreachable!("StopWorker is handled in run()"),
    }
}
