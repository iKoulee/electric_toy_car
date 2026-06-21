#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum LinkState {
    AwaitingFirstPacket,
    Alive,
    TimedOut,
}

#[derive(Copy, Clone, Debug)]
pub struct LinkWatchdog {
    timeout_ms: u64,
    last_valid_rx_ms: Option<u64>,
}

impl LinkWatchdog {
    pub const fn new(timeout_ms: u64) -> Self {
        Self {
            timeout_ms,
            last_valid_rx_ms: None,
        }
    }

    pub fn record_valid_packet(&mut self, now_ms: u64) {
        self.last_valid_rx_ms = Some(now_ms);
    }

    pub fn state(&self, now_ms: u64) -> LinkState {
        match self.last_valid_rx_ms {
            Some(last_rx) => {
                if now_ms.saturating_sub(last_rx) > self.timeout_ms {
                    LinkState::TimedOut
                } else {
                    LinkState::Alive
                }
            }
            None => {
                if now_ms > self.timeout_ms {
                    LinkState::TimedOut
                } else {
                    LinkState::AwaitingFirstPacket
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn times_out_without_packets() {
        let watchdog = LinkWatchdog::new(500);
        assert_eq!(watchdog.state(100), LinkState::AwaitingFirstPacket);
        assert_eq!(watchdog.state(600), LinkState::TimedOut);
    }

    #[test]
    fn recovers_after_valid_packet() {
        let mut watchdog = LinkWatchdog::new(500);
        watchdog.record_valid_packet(1000);
        assert_eq!(watchdog.state(1200), LinkState::Alive);
        assert_eq!(watchdog.state(1601), LinkState::TimedOut);

        watchdog.record_valid_packet(1700);
        assert_eq!(watchdog.state(1701), LinkState::Alive);
    }
}
