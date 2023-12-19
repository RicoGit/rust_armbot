/// Re-maps a number from one range to another.
pub fn map(from: u32, from_min: u32, from_max: u32, to_min: u32, to_max: u32, invert: bool) -> u32 {
    let to_range = to_max - to_min;
    let from_range = from_max - from_min;
    let ratio = to_range as f32 / from_range as f32;
    let from = from.min(from_max).max(from_min);

    let to = ((from - from_min) as f32 * ratio) as u32;

    if invert {
        to_max - to
    } else {
        to + to_min
    }
}
