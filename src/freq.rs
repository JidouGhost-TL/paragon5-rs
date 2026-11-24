//! Fixed-point 16.16 pitch stepping, volume scaling, and GBA Timer0 rate utilities for
//! the Paragon 5 / GBASS Sound Engine.

/// GBA master CPU clock frequency in Hz (`2^24 = 16,777,216 Hz`).
pub const GBA_CLOCK_HZ: u32 = 16_777_216;

/// Unity 16.16 fixed-point step / pitch multiplier (`1.0 = 0x0001_0000`).
pub const Q16_ONE: u32 = 0x0001_0000;

/// Default Paragon 5 / GBASS DirectSound output mix rate in Hz (`13,379 Hz`).
pub const PARAGON5_DEFAULT_MIX_RATE_HZ: u32 = 13_379;

/// Number of PCM samples produced per GBASS voice-pair or summing mixer block (`64`).
pub const PARAGON5_BLOCK_SIZE: usize = 64;

/// Maximum number of 64-byte voice-pair track buffers summed per output channel (`8`).
pub const PARAGON5_MAX_TRACKS: usize = 8;

/// Convert a desired playback frequency in Hz to a 16.16 fixed-point step increment
/// for the given hardware mix rate.
#[inline]
#[must_use]
pub fn freq_to_step_q16(freq_hz: f64, mix_rate_hz: f64) -> u32 {
    if !freq_hz.is_finite() || !mix_rate_hz.is_finite() || freq_hz <= 0.0 || mix_rate_hz <= 0.0 {
        return 0;
    }
    let scaled = (freq_hz / mix_rate_hz) * (Q16_ONE as f64);
    if scaled >= (u32::MAX as f64) {
        u32::MAX
    } else {
        (scaled + 0.5) as u32
    }
}

/// Convert a 16.16 fixed-point step increment back to a playback frequency in Hz.
#[inline]
#[must_use]
pub fn step_q16_to_freq(step_q16: u32, mix_rate_hz: f64) -> f64 {
    ((step_q16 as f64) / (Q16_ONE as f64)) * mix_rate_hz
}

/// Compute the exact ARM7TDMI effective 16.16 step increment:
/// `smull(pitch_scale_fp16, base_step_fp16) >> 16`.
///
/// Mirrors the exact GBASS ARM sequence:
/// ```text
/// smull r6, sp, ip, r6
/// lsr   r6, r6, #16
/// orr   r6, r6, sp, lsl #16
/// ```
#[inline]
#[must_use]
pub const fn effective_step_q16(base_step_q16: u32, pitch_scale_q16: i32) -> u32 {
    let prod = (pitch_scale_q16 as i64).wrapping_mul((base_step_q16 as i32) as i64);
    ((prod as u64) >> 16) as u32
}

/// Compute the exact ARM7TDMI 3-stage effective voice volume:
/// `(((volume * vol_scale_a) * vol_scale_b) as i32) >> 16`.
///
/// Mirrors the exact GBASS ARM sequence:
/// ```text
/// ldrh ip, [r0, #0x18]
/// mul  sp, ip, r3
/// ldrh ip, [r0, #0x1a]
/// mul  r3, ip, sp
/// asr  r3, r3, #16
/// ```
#[inline]
#[must_use]
pub const fn effective_volume(volume: u32, vol_scale_a: u16, vol_scale_b: u16) -> i32 {
    let stage1 = volume.wrapping_mul(vol_scale_a as u32);
    let stage2 = stage1.wrapping_mul(vol_scale_b as u32);
    (stage2 as i32) >> 16
}

/// Compute the DirectSound sample rate in Hz from a GBA Timer0 reload value (`TM0CNT_L`).
#[inline]
#[must_use]
pub const fn rate_from_timer_reload(reload: u16) -> u32 {
    let ticks = 65_536u32 - (reload as u32);
    (GBA_CLOCK_HZ + (ticks >> 1)) / ticks
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_step_and_rate_helpers() {
        assert_eq!(rate_from_timer_reload(0xFB1A), 13_379);
        assert_eq!(rate_from_timer_reload(0xF9C4), 10_512);
        assert_eq!(rate_from_timer_reload(0xFCE2), 21_024);

        assert_eq!(freq_to_step_q16(13_379.0, 13_379.0), Q16_ONE);
        assert!((step_q16_to_freq(Q16_ONE, 13_379.0) - 13_379.0).abs() < 1e-6);

        assert_eq!(effective_step_q16(0x0001_4000, 0x0001_2000), 0x0001_6800);
        assert_eq!(effective_volume(0x0180, 0x0100, 0x00C0), 0x0120);
    }
}
