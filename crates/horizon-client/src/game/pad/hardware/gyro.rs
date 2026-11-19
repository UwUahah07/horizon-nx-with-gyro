//! The pad's gyroscope (Switch six-axis sensor via horizon_nx::gyro).

#[cfg_attr(not(target_os = "horizon"), allow(dead_code))]
#[derive(Clone, Copy, Debug, PartialEq)]
struct Axes {
    yaw: (usize, f32),
    pitch: (usize, f32),
}

#[cfg_attr(not(target_os = "horizon"), allow(dead_code))]
impl Axes {
    const DEFAULT: Axes = Axes {
        yaw: (2, 1.0),
        pitch: (0, 1.0),
    };

    fn parse(text: &str) -> Option<Axes> {
        let (yaw, pitch) = text.split_once(',')?;
        let one = |t: &str| {
            let t = t.trim();
            let (sign, t) = match t.strip_prefix('-') {
                Some(rest) => (-1.0, rest),
                None => (1.0, t),
            };
            let axis = match t.trim() {
                "x" | "X" => 0,
                "y" | "Y" => 1,
                "z" | "Z" => 2,
                _ => return None,
            };
            Some((axis, sign))
        };
        let (yaw, pitch) = (one(yaw)?, one(pitch)?);
        (yaw.0 != pitch.0).then_some(Axes { yaw, pitch })
    }

    fn map(self, v: [f32; 3]) -> [f32; 2] {
        [v[self.yaw.0] * self.yaw.1, v[self.pitch.0] * self.pitch.1]
    }
}

#[cfg(target_os = "horizon")]
pub(super) use switch::Sensor;
#[cfg(not(target_os = "horizon"))]
pub(super) use none::Sensor;

#[cfg(not(target_os = "horizon"))]
mod none {
    #[derive(Default)]
    pub(in crate::game) struct Sensor;
    impl Sensor {
        pub(in crate::game) fn rate(&mut self) -> [f32; 2] {
            [0.0, 0.0]
        }
    }
}

#[cfg(target_os = "horizon")]
mod switch {
    use super::Axes;
    use horizon_nx::gyro::{self, Handle};
    use std::time::{Duration, Instant};

    const SOURCES: [(u32, u32, usize); 3] = [(0x20, 1 << 1, 1), (0, 1 << 0, 1), (0, 1 << 2, 2)];
    const SAMPLING_NUMBER: usize = 8;
    const ANGULAR_VELOCITY: usize = 28;
    const DEGREES_PER_UNIT: f32 = 360.0;
    const STALE: Duration = Duration::from_millis(100);
    const RETRY: Duration = Duration::from_secs(2);

    #[derive(Default)]
    pub(in crate::game) struct Sensor {
        handles: Vec<Handle>,
        last: Vec<u64>,
        retry_at: Option<Instant>,
        axes: Option<Axes>,
        velocity: [f32; 3],
        fresh_at: Option<Instant>,
    }

    impl Sensor {
        fn start(&mut self) {
            self.handles.clear();
            for (id, style, count) in SOURCES {
                self.handles.extend(gyro::start_handles(id, style, count));
            }
            self.last = vec![0; self.handles.len()];
        }

        pub(in crate::game) fn rate(&mut self) -> [f32; 2] {
            let now = Instant::now();
            if self.handles.is_empty() {
                if self.retry_at.is_some_and(|at| now < at) {
                    return [0.0, 0.0];
                }
                self.retry_at = Some(now + RETRY);
                self.start();
                if self.handles.is_empty() {
                    return [0.0, 0.0];
                }
            }
            let mut best: Option<([f32; 3], f32)> = None;
            for (i, &handle) in self.handles.iter().enumerate() {
                let mut raw = [0u64; 32];
                if gyro::read_state(handle, &mut raw) == 0 {
                    continue;
                }
                let mut bytes = [0u8; 256];
                for (idx, w) in raw.iter().enumerate() {
                    bytes[idx * 8..(idx + 1) * 8].copy_from_slice(&w.to_ne_bytes());
                }
                let sampling = u64::from_ne_bytes(
                    bytes[SAMPLING_NUMBER..SAMPLING_NUMBER + 8].try_into().unwrap(),
                );
                if sampling == self.last[i] {
                    continue;
                }
                self.last[i] = sampling;
                let float = |n: usize| {
                    let off = ANGULAR_VELOCITY + 4 * n;
                    f32::from_ne_bytes(bytes[off..off + 4].try_into().unwrap())
                };
                let v = [float(0), float(1), float(2)];
                if !v.iter().all(|c| c.is_finite()) {
                    continue;
                }
                let size = v.iter().map(|c| c * c).sum::<f32>();
                if best.is_none_or(|(_, b)| size > b) {
                    best = Some((v, size));
                }
            }
            if let Some((v, _)) = best {
                self.velocity = v.map(|c| c * DEGREES_PER_UNIT);
                self.fresh_at = Some(now);
            }
            if self.fresh_at.is_none_or(|at| now.duration_since(at) > STALE) {
                return [0.0, 0.0];
            }
            let axes = *self.axes.get_or_insert_with(|| {
                std::env::var("HORIZON_GYRO_AXES")
                    .ok()
                    .and_then(|text| Axes::parse(&text))
                    .unwrap_or(Axes::DEFAULT)
            });
            axes.map(self.velocity)
        }
    }
}
