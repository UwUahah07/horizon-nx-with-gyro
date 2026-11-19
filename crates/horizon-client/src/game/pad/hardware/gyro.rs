//! The pad's gyroscope. gilrs has no motion sensors, so on the Switch this reads libnx's
//! six-axis sensor (`switch/services/hid.h`) next to gilrs' npad; everywhere else it reads zero.
//!
//! `Sensor::rate` gives the angular speed in degrees per second as `[yaw, pitch]` (yaw positive
//! turning right, pitch positive tilting the top of the pad up), the pair `gamepad::gyro_delta`
//! takes. Which sensor axis is yaw and which is pitch depends on how the console reports them:
//! `HORIZON_GYRO_AXES=yaw,pitch` in env.txt next to the NRO picks them without a rebuild
//! (`z,x` is the default; `-z,x`, `y,x`, `-y,-x`... are the other tries). The sign of either can
//! also be flipped with `gyroInvertX` / `gyroInvertY` in horizon-controller.txt.

/// Which sensor axis (0 = x, 1 = y, 2 = z) drives yaw and pitch, each with a sign.
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

    /// `"z,x"`, `"-y,x"`: yaw's axis, then pitch's, each with an optional minus.
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
    use std::time::{Duration, Instant};

    /// libnx `HidSixAxisSensorHandle`: one 32-bit word.
    type Handle = u32;

    unsafe extern "C" {
        fn hidGetSixAxisSensorHandles(
            handles: *mut Handle,
            total_handle_count: i32,
            id: u32,
            style: u32,
        ) -> u32;
        fn hidStartSixAxisSensor(handle: Handle) -> u32;
        fn hidGetSixAxisSensorStates(handle: Handle, states: *mut u8, count: usize) -> usize;
    }

    /// The npads whose sensors are read: libnx's `HidNpadIdType` (No1 = 0, Handheld = 0x20), its
    /// `HidNpadStyleTag` (FullKey 1, Handheld 2, JoyDual 4) and how many handles the style has
    /// (a pair of Joy-Con has one per Joy-Con).
    const SOURCES: [(u32, u32, i32); 3] = [(0x20, 1 << 1, 1), (0, 1 << 0, 1), (0, 1 << 2, 2)];

    /// Room for one `HidSixAxisSensorState`, whatever size this libnx makes it: only its first
    /// fields are read, which every libnx version lays out the same: `delta_time` (u64),
    /// `sampling_number` (u64), `acceleration` (3 f32), `angular_velocity` (3 f32, at byte 28).
    #[repr(C, align(8))]
    struct Raw([u8; 256]);
    const SAMPLING_NUMBER: usize = 8;
    const ANGULAR_VELOCITY: usize = 28;

    /// The sensor reports revolutions per second; the game wants degrees.
    const DEGREES_PER_UNIT: f32 = 360.0;
    /// With no new sample for this long the pad is gone or asleep: the view stops.
    const STALE: Duration = Duration::from_millis(100);
    /// How often to look again for sensors while none were found (no pad yet).
    const RETRY: Duration = Duration::from_secs(2);

    #[derive(Default)]
    pub(in crate::game) struct Sensor {
        handles: Vec<Handle>,
        /// Each handle's last `sampling_number`: a sensor of a pad that isn't in use stands still.
        last: Vec<u64>,
        retry_at: Option<Instant>,
        axes: Option<Axes>,
        /// The last angular velocity (sensor axes, degrees per second) and when it arrived.
        velocity: [f32; 3],
        fresh_at: Option<Instant>,
    }

    impl Sensor {
        fn start(&mut self) {
            self.handles.clear();
            for (id, style, count) in SOURCES {
                let mut handles = [0 as Handle; 2];
                // SAFETY: libnx fills `count` (at most 2) handles; hid is up (gilrs' npad needs it).
                if unsafe { hidGetSixAxisSensorHandles(handles.as_mut_ptr(), count, id, style) } != 0 {
                    continue;
                }
                for &handle in &handles[..count as usize] {
                    // A sensor that doesn't start is read as standing still.
                    let _ = unsafe { hidStartSixAxisSensor(handle) };
                    self.handles.push(handle);
                }
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
                let mut raw = Raw([0; 256]);
                // SAFETY: `raw` is larger than a state; one state is asked for.
                if unsafe { hidGetSixAxisSensorStates(handle, raw.0.as_mut_ptr(), 1) } == 0 {
                    continue;
                }
                let word = |at: usize, len: usize| &raw.0[at..at + len];
                let sampling = u64::from_ne_bytes(word(SAMPLING_NUMBER, 8).try_into().unwrap());
                if sampling == self.last[i] {
                    continue;
                }
                self.last[i] = sampling;
                let float = |n: usize| {
                    f32::from_ne_bytes(word(ANGULAR_VELOCITY + 4 * n, 4).try_into().unwrap())
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

#[cfg(test)]
mod tests {
    use super::Axes;

    #[test]
    fn axes_parse_and_map() {
        assert_eq!(Axes::parse("z,x"), Some(Axes::DEFAULT));
        let axes = Axes::parse(" -y , x ").unwrap();
        assert_eq!(axes.map([1.0, 2.0, 3.0]), [-2.0, 1.0]);
        // The same axis can't be both, and junk is refused.
        assert_eq!(Axes::parse("x,-x"), None);
        assert_eq!(Axes::parse("w,x"), None);
        assert_eq!(Axes::parse("z"), None);
    }
}
