//! Nintendo Switch six-axis sensor (gyroscope) via libnx.

pub type Handle = u32;

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

pub fn start_handles(id: u32, style: u32, count: usize) -> Vec<Handle> {
    let mut buf = [0 as Handle; 2];
    let n = count.min(buf.len());
    if unsafe { hidGetSixAxisSensorHandles(buf.as_mut_ptr(), n as i32, id, style) } != 0 {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(n);
    for &handle in &buf[..n] {
        let _ = unsafe { hidStartSixAxisSensor(handle) };
        out.push(handle);
    }
    out
}

pub fn read_state(handle: Handle, raw: &mut [u64; 32]) -> usize {
    unsafe { hidGetSixAxisSensorStates(handle, raw.as_mut_ptr().cast::<u8>(), 1) }
}
