//! Paragon 5 / GBASS 32-byte ARM7TDMI voice state structure.

use crate::freq::{effective_step_q16, effective_volume, Q16_ONE};
use crate::sample::Paragon5SampleDesc;

/// Exact 32-byte ARM7TDMI voice state consumed by the Paragon 5 / GBASS voice-pair mixer.
///
/// Mirrors the in-memory GBA voice struct loaded via `ldm r0, {r2, r3, r4, r5, r6}` plus
/// offsets `+0x14`, `+0x18`, `+0x1A`, `+0x1C`:
/// - `+0x00`: `sample_ptr` (passed separately as `pcm: &[u8]` in safe Rust)
/// - `+0x04`: `vol` (`u32`)
/// - `+0x08`: `pos_fp16` (`u32`, updated in-place after every 64-sample block)
/// - `+0x0C`: `end_fp16` (`u32`, unsigned loop/end boundary)
/// - `+0x10`: `step_fp16` (`u32`, base 16.16 step before `pitch_scale_fp16`)
/// - `+0x14`: `loop_len_fp16` (`u32`; if `loop_len_fp16 == step_fp16`, the mixer enters
///   one-shot freeze-at-end mode by subtracting `effective_step_fp16` upon reaching `end_fp16`)
/// - `+0x18`: `vol_scale_a` (`u16`, channel/master volume multiplier)
/// - `+0x1A`: `vol_scale_b` (`u16`, pan/fade volume multiplier)
/// - `+0x1C`: `pitch_scale_fp16` (`i32`, signed 16.16 pitch multiplier applied via `smull >> 16`)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Paragon5Voice {
    /// Base voice volume (`+0x04`).
    pub vol: u32,
    /// Current 16.16 fixed-point sample position (`+0x08`).
    pub pos_fp16: u32,
    /// End / loop-end boundary in 16.16 fixed-point (`+0x0C`).
    pub end_fp16: u32,
    /// Base 16.16 fixed-point step increment (`+0x10`).
    pub step_fp16: u32,
    /// Loop subtraction length in 16.16 fixed-point (`+0x14`).
    ///
    /// When `loop_len_fp16 == step_fp16`, the ARM mixer subtracts `effective_step_fp16`
    /// whenever `pos_fp16 >= end_fp16`, freezing playback on the terminal sample.
    pub loop_len_fp16: u32,
    /// First 16-bit volume scale factor (`+0x18`, e.g. `0x0100` = 1.0x).
    pub vol_scale_a: u16,
    /// Second 16-bit volume scale factor (`+0x1A`, e.g. `0x0100` = 1.0x).
    pub vol_scale_b: u16,
    /// Signed 16.16 pitch scale multiplier (`+0x1C`, `0x0001_0000` = 1.0x).
    pub pitch_scale_fp16: i32,
}

impl Paragon5Voice {
    /// Create a looped voice with explicit 16.16 fixed-point parameters.
    #[inline]
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub const fn new_looped(
        vol: u32,
        pos_fp16: u32,
        end_fp16: u32,
        step_fp16: u32,
        loop_len_fp16: u32,
        vol_scale_a: u16,
        vol_scale_b: u16,
        pitch_scale_fp16: i32,
    ) -> Self {
        Self {
            vol,
            pos_fp16,
            end_fp16,
            step_fp16,
            loop_len_fp16,
            vol_scale_a,
            vol_scale_b,
            pitch_scale_fp16,
        }
    }

    /// Create a one-shot voice using the exact GBASS freeze-at-end convention
    /// (`loop_len_fp16 = step_fp16`).
    #[inline]
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub const fn new_oneshot(
        vol: u32,
        pos_fp16: u32,
        end_fp16: u32,
        step_fp16: u32,
        vol_scale_a: u16,
        vol_scale_b: u16,
        pitch_scale_fp16: i32,
    ) -> Self {
        Self {
            vol,
            pos_fp16,
            end_fp16,
            step_fp16,
            loop_len_fp16: step_fp16,
            vol_scale_a,
            vol_scale_b,
            pitch_scale_fp16,
        }
    }

    /// Create a voice initialized from a parsed [`Paragon5SampleDesc`].
    #[inline]
    #[must_use]
    pub const fn from_sample_desc(
        desc: &Paragon5SampleDesc,
        vol: u32,
        step_fp16: u32,
        vol_scale_a: u16,
        vol_scale_b: u16,
    ) -> Self {
        let (end_fp16, loop_len_fp16) = if desc.is_looped() {
            let end = if desc.loop_end_fp16 <= desc.length_fp16 {
                desc.loop_end_fp16
            } else {
                desc.length_fp16
            };
            (end, end.saturating_sub(desc.loop_start_fp16))
        } else {
            (desc.length_fp16, step_fp16)
        };
        Self {
            vol,
            pos_fp16: 0,
            end_fp16,
            step_fp16,
            loop_len_fp16,
            vol_scale_a,
            vol_scale_b,
            pitch_scale_fp16: desc.pitch_scale_fp16 as i32,
        }
    }

    /// Create a silent dummy voice (volume `0`, frozen at index `0`).
    #[inline]
    #[must_use]
    pub const fn silent() -> Self {
        Self {
            vol: 0,
            pos_fp16: 0,
            end_fp16: Q16_ONE,
            step_fp16: 0,
            loop_len_fp16: 0,
            vol_scale_a: 0,
            vol_scale_b: 0,
            pitch_scale_fp16: Q16_ONE as i32,
        }
    }

    /// Returns `true` if this voice triggers the GBASS one-shot freeze-at-end path
    /// (`loop_len_fp16 == step_fp16`).
    #[inline]
    #[must_use]
    pub const fn is_oneshot_freeze(&self) -> bool {
        self.loop_len_fp16 == self.step_fp16
    }

    /// Compute the exact ARM7TDMI effective voice volume (`((vol * vol_scale_a) * vol_scale_b) >> 16`).
    #[inline]
    #[must_use]
    pub const fn effective_volume(&self) -> i32 {
        effective_volume(self.vol, self.vol_scale_a, self.vol_scale_b)
    }

    /// Compute the exact ARM7TDMI effective 16.16 step (`smull(pitch_scale_fp16, step_fp16) >> 16`).
    #[inline]
    #[must_use]
    pub const fn effective_step_fp16(&self) -> u32 {
        effective_step_q16(self.step_fp16, self.pitch_scale_fp16)
    }
}

impl Default for Paragon5Voice {
    #[inline]
    fn default() -> Self {
        Self::silent()
    }
}
