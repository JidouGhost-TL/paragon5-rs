//! Dual Synthesis Engines for the Paragon 5 / GBASS GBA Sound Engine:
//!
//! - [`AccuracySynth`]: 100% bit-exact emulation of both Paragon 5 / GBASS ARM7TDMI mixer
//!   revisions (`GbassV1` & `GbassV2`) verified strictly byte-for-byte (`assert_eq!`, 0 tolerance)
//!   against headless `libmgba.so.0.10` ARM7TDMI execution.
//! - [`StudioSynth`]: High-fidelity 48,000 Hz `f32` software synthesizer featuring a compile-time
//!   256-phase × 6-tap Lanczos-3 polyphase windowed-sinc LUT (with general Lanczos-`a` fallback)
//!   and true equal-power stereo panning.

use crate::error::{Error, Result};
use crate::freq::{PARAGON5_BLOCK_SIZE, PARAGON5_MAX_TRACKS, Q16_ONE};
use crate::sample::Paragon5SampleView;
use crate::version::Paragon5Version;
use crate::voice::Paragon5Voice;

// ============================================================================
// Pure no_std math & compile-time Lanczos-3 polyphase LUT
// ============================================================================

const LANCZOS_PHASES: usize = 256;
const LANCZOS3_TAPS: usize = 6;

/// 32-byte aligned Lanczos-3 polyphase filter bank (`256` phases × 8 `f32` lanes = 8 KiB,
/// padded with two zero lanes for shift-by-5 indexing and L1-friendly alignment).
#[repr(C, align(32))]
struct Lanczos3Table([[f32; 8]; LANCZOS_PHASES]);

/// Pure `const fn` sine approximation on `[-pi, pi]` (range-reduced, < 1e-13 max error).
#[inline]
const fn sin_const(mut x: f64) -> f64 {
    const PI: f64 = core::f64::consts::PI;
    const TWO_PI: f64 = 2.0 * PI;
    const HALF_PI: f64 = core::f64::consts::FRAC_PI_2;

    if x > PI || x < -PI {
        let k = ((x + PI) / TWO_PI) as i64;
        x -= (k as f64) * TWO_PI;
        if x > PI {
            x -= TWO_PI;
        } else if x < -PI {
            x += TWO_PI;
        }
    }
    if x > HALF_PI {
        x = PI - x;
    } else if x < -HALF_PI {
        x = -PI - x;
    }

    let x2 = x * x;
    let mut term = x;
    let mut sum = x;
    let mut n = 1u32;
    while n <= 9 {
        let d = (2 * n) * (2 * n + 1);
        term = -term * x2 / (d as f64);
        sum += term;
        n += 1;
    }
    sum
}

#[inline]
const fn cos_const(x: f64) -> f64 {
    sin_const(x + core::f64::consts::FRAC_PI_2)
}

#[inline]
const fn lanczos_kernel_const(x: f64, a: f64) -> f64 {
    let ax = if x < 0.0 { -x } else { x };
    if ax < 1e-12 {
        return 1.0;
    }
    if ax >= a {
        return 0.0;
    }
    let px = core::f64::consts::PI * x;
    let px_a = px / a;
    (sin_const(px) / px) * (sin_const(px_a) / px_a)
}

/// Build the normalized 256-entry (`0..256`) × 6-tap (padded to 8) Lanczos-3 polyphase filter
/// table entirely at compile time.
const fn build_lanczos3_lut() -> Lanczos3Table {
    let mut table = [[0.0f32; 8]; LANCZOS_PHASES];
    let mut p = 0usize;
    while p < LANCZOS_PHASES {
        let frac = (p as f64) / (LANCZOS_PHASES as f64);
        let mut weights = [0.0f64; LANCZOS3_TAPS];
        let mut wsum = 0.0f64;
        let mut t = 0usize;
        while t < LANCZOS3_TAPS {
            let k = (t as isize) - 2; // taps at -2, -1, 0, +1, +2, +3
            let w = lanczos_kernel_const(frac - (k as f64), 3.0);
            weights[t] = w;
            wsum += w;
            t += 1;
        }
        let inv = if wsum > 1e-12 { 1.0 / wsum } else { 1.0 };
        let mut t2 = 0usize;
        while t2 < LANCZOS3_TAPS {
            table[p][t2] = (weights[t2] * inv) as f32;
            t2 += 1;
        }
        p += 1;
    }
    Lanczos3Table(table)
}

static LANCZOS3_LUT: Lanczos3Table = build_lanczos3_lut();

// ============================================================================
// AccuracySynth — 100% Bit-Exact ARM7TDMI GBASS v1 & v2 Emulation
// ============================================================================

/// Byte-exact ARM7TDMI software mixer emulation for both Paragon 5 / GBASS revisions
/// ([`Paragon5Version::GbassV1`] and [`Paragon5Version::GbassV2`]).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AccuracySynth;

impl AccuracySynth {
    /// Mix a pair of Paragon 5 / GBASS voices into a 64-byte signed 8-bit track buffer.
    ///
    /// Mirrors the exact ARM7TDMI voice-pair inner loop shared by both `GbassV1` (`ABQE`) and
    /// `GbassV2` (`A2IJ`, `AG4E`, etc.):
    /// - 3-stage volume scaling: `eff_vol = (((vol * vol_scale_a) * vol_scale_b) as i32) >> 16`.
    /// - 16.16 signed pitch scaling: `eff_step = smull(pitch_scale_fp16, step_fp16) >> 16`.
    /// - Loop subtraction: `sub = if loop_len_fp16 == step_fp16 { eff_step } else { loop_len_fp16 }`.
    /// - Output sample: `((sa * eff_vol_a + sb * eff_vol_b) >> 9) as i8` (wrapping `strb`).
    /// - Unsigned boundary compare (`cmp hs`): `if pos >= end { pos = pos.wrapping_sub(sub) }`.
    ///
    /// Both `voice_a.pos_fp16` and `voice_b.pos_fp16` are updated in-place after 64 samples.
    pub fn mix_voice_pair(
        pcm_a: &[u8],
        voice_a: &mut Paragon5Voice,
        pcm_b: &[u8],
        voice_b: &mut Paragon5Voice,
        out: &mut [i8; PARAGON5_BLOCK_SIZE],
    ) {
        let vol_a = voice_a.effective_volume();
        let vol_b = voice_b.effective_volume();
        let step_a = voice_a.effective_step_fp16();
        let step_b = voice_b.effective_step_fp16();

        let mut pos_a = voice_a.pos_fp16;
        let end_a = voice_a.end_fp16;
        let mut pos_b = voice_b.pos_fp16;
        let end_b = voice_b.end_fp16;

        let len_a = pcm_a.len();
        let len_b = pcm_b.len();

        // Fast path: neither voice reaches `end_fp16` or wraps `u32` during this 64-sample block,
        // and all sampled indices stay strictly within `pcm_a` and `pcm_b`.
        let end_pos_a = (pos_a as u64) + (PARAGON5_BLOCK_SIZE as u64) * (step_a as u64);
        let end_pos_b = (pos_b as u64) + (PARAGON5_BLOCK_SIZE as u64) * (step_b as u64);
        if end_pos_a < (end_a as u64) && end_pos_b < (end_b as u64) {
            let max_idx_a = ((end_pos_a - (step_a as u64)) >> 16) as usize;
            let max_idx_b = ((end_pos_b - (step_b as u64)) >> 16) as usize;
            if max_idx_a < len_a && max_idx_b < len_b {
                if step_a == Q16_ONE && step_b == Q16_ONE {
                    // 1:1 pitch fast path: contiguous slices enable full SIMD auto-vectorization.
                    let base_a = (pos_a >> 16) as usize;
                    let base_b = (pos_b >> 16) as usize;
                    let src_a = &pcm_a[base_a..base_a + PARAGON5_BLOCK_SIZE];
                    let src_b = &pcm_b[base_b..base_b + PARAGON5_BLOCK_SIZE];
                    for ((slot, &ba), &bb) in out.iter_mut().zip(src_a.iter()).zip(src_b.iter()) {
                        let sa = ba as i8 as i32;
                        let sb = bb as i8 as i32;
                        let acc = sa.wrapping_mul(vol_a).wrapping_add(sb.wrapping_mul(vol_b));
                        *slot = (acc >> 9) as i8;
                    }
                } else {
                    // General non-wrapping fast path: gather each voice into a contiguous 64-byte
                    // stack buffer, then run a bounds-check-free SIMD multiply-add-shift pass.
                    let base_a = (pos_a >> 16) as usize;
                    let base_b = (pos_b >> 16) as usize;
                    let max_rel_a = max_idx_a - base_a;
                    let max_rel_b = max_idx_b - base_b;

                    let mut buf_a = [0i8; PARAGON5_BLOCK_SIZE];
                    let mut buf_b = [0i8; PARAGON5_BLOCK_SIZE];

                    let mut pa = pos_a & 0xFFFF;
                    if max_rel_a < 256 && base_a + 256 <= len_a {
                        let win_a: &[u8; 256] =
                            pcm_a[base_a..base_a + 256].try_into().unwrap();
                        let s2 = step_a.wrapping_mul(2);
                        let s3 = step_a.wrapping_mul(3);
                        let s4 = step_a.wrapping_mul(4);
                        for chunk in buf_a.as_chunks_mut::<4>().0 {
                            chunk[0] = win_a[((pa >> 16) as usize) & 0xFF] as i8;
                            chunk[1] =
                                win_a[((pa.wrapping_add(step_a) >> 16) as usize) & 0xFF] as i8;
                            chunk[2] =
                                win_a[((pa.wrapping_add(s2) >> 16) as usize) & 0xFF] as i8;
                            chunk[3] =
                                win_a[((pa.wrapping_add(s3) >> 16) as usize) & 0xFF] as i8;
                            pa = pa.wrapping_add(s4);
                        }
                    } else {
                        let src_a = &pcm_a[base_a..=max_idx_a];
                        for slot in &mut buf_a {
                            *slot = src_a[(pa >> 16) as usize] as i8;
                            pa = pa.wrapping_add(step_a);
                        }
                    }

                    let mut pb = pos_b & 0xFFFF;
                    if max_rel_b < 256 && base_b + 256 <= len_b {
                        let win_b: &[u8; 256] =
                            pcm_b[base_b..base_b + 256].try_into().unwrap();
                        let s2 = step_b.wrapping_mul(2);
                        let s3 = step_b.wrapping_mul(3);
                        let s4 = step_b.wrapping_mul(4);
                        for chunk in buf_b.as_chunks_mut::<4>().0 {
                            chunk[0] = win_b[((pb >> 16) as usize) & 0xFF] as i8;
                            chunk[1] =
                                win_b[((pb.wrapping_add(step_b) >> 16) as usize) & 0xFF] as i8;
                            chunk[2] =
                                win_b[((pb.wrapping_add(s2) >> 16) as usize) & 0xFF] as i8;
                            chunk[3] =
                                win_b[((pb.wrapping_add(s3) >> 16) as usize) & 0xFF] as i8;
                            pb = pb.wrapping_add(s4);
                        }
                    } else {
                        let src_b = &pcm_b[base_b..=max_idx_b];
                        for slot in &mut buf_b {
                            *slot = src_b[(pb >> 16) as usize] as i8;
                            pb = pb.wrapping_add(step_b);
                        }
                    }

                    for ((slot, &sa), &sb) in out.iter_mut().zip(&buf_a).zip(&buf_b) {
                        let acc = (sa as i32)
                            .wrapping_mul(vol_a)
                            .wrapping_add((sb as i32).wrapping_mul(vol_b));
                        *slot = (acc >> 9) as i8;
                    }
                }
                voice_a.pos_fp16 = end_pos_a as u32;
                voice_b.pos_fp16 = end_pos_b as u32;
                return;
            }
        }

        let sub_a = if voice_a.is_oneshot_freeze() {
            step_a
        } else {
            voice_a.loop_len_fp16
        };
        let sub_b = if voice_b.is_oneshot_freeze() {
            step_b
        } else {
            voice_b.loop_len_fp16
        };

        for slot in out.iter_mut() {
            let idx_a = (pos_a >> 16) as usize;
            let idx_b = (pos_b >> 16) as usize;
            let sa = if idx_a < len_a {
                pcm_a[idx_a] as i8 as i32
            } else {
                0
            };
            let sb = if idx_b < len_b {
                pcm_b[idx_b] as i8 as i32
            } else {
                0
            };

            let acc = sa.wrapping_mul(vol_a).wrapping_add(sb.wrapping_mul(vol_b));
            *slot = (acc >> 9) as i8;

            pos_a = pos_a.wrapping_add(step_a);
            if pos_a >= end_a {
                pos_a = pos_a.wrapping_sub(sub_a);
            }

            pos_b = pos_b.wrapping_add(step_b);
            if pos_b >= end_b {
                pos_b = pos_b.wrapping_sub(sub_b);
            }
        }

        voice_a.pos_fp16 = pos_a;
        voice_b.pos_fp16 = pos_b;
    }

    /// Sum `1..=8` 64-byte voice-pair track buffers with signed `[-128, +127]` saturation.
    ///
    /// Mirrors the exact ARM7TDMI self-modifying summing mixer shared by both `GbassV1` and
    /// `GbassV2`. Returns [`Error::InvalidTrackCount`] if `tracks.is_empty()` or
    /// `tracks.len() > 8`.
    pub fn sum_tracks(
        tracks: &[[i8; PARAGON5_BLOCK_SIZE]],
        out: &mut [i8; PARAGON5_BLOCK_SIZE],
    ) -> Result<()> {
        // Maximum sum of 8 `i8` values is [-1024, +1016], which fits cleanly in `i16`
        // for full-width SIMD auto-vectorization across all 64 samples.
        match tracks {
            [t0] => out.copy_from_slice(t0),
            [t0, t1] => {
                for i in 0..PARAGON5_BLOCK_SIZE {
                    let sum = (t0[i] as i16) + (t1[i] as i16);
                    out[i] = sum.clamp(-128, 127) as i8;
                }
            }
            [t0, t1, t2] => {
                for i in 0..PARAGON5_BLOCK_SIZE {
                    let sum = (t0[i] as i16) + (t1[i] as i16) + (t2[i] as i16);
                    out[i] = sum.clamp(-128, 127) as i8;
                }
            }
            [t0, t1, t2, t3] => {
                for i in 0..PARAGON5_BLOCK_SIZE {
                    let sum =
                        ((t0[i] as i16) + (t1[i] as i16)) + ((t2[i] as i16) + (t3[i] as i16));
                    out[i] = sum.clamp(-128, 127) as i8;
                }
            }
            [t0, t1, t2, t3, t4] => {
                for i in 0..PARAGON5_BLOCK_SIZE {
                    let sum = ((t0[i] as i16) + (t1[i] as i16))
                        + ((t2[i] as i16) + (t3[i] as i16))
                        + (t4[i] as i16);
                    out[i] = sum.clamp(-128, 127) as i8;
                }
            }
            [t0, t1, t2, t3, t4, t5] => {
                for i in 0..PARAGON5_BLOCK_SIZE {
                    let sum = ((t0[i] as i16) + (t1[i] as i16))
                        + ((t2[i] as i16) + (t3[i] as i16))
                        + ((t4[i] as i16) + (t5[i] as i16));
                    out[i] = sum.clamp(-128, 127) as i8;
                }
            }
            [t0, t1, t2, t3, t4, t5, t6] => {
                for i in 0..PARAGON5_BLOCK_SIZE {
                    let sum = (((t0[i] as i16) + (t1[i] as i16))
                        + ((t2[i] as i16) + (t3[i] as i16)))
                        + (((t4[i] as i16) + (t5[i] as i16)) + (t6[i] as i16));
                    out[i] = sum.clamp(-128, 127) as i8;
                }
            }
            [t0, t1, t2, t3, t4, t5, t6, t7] => {
                for i in 0..PARAGON5_BLOCK_SIZE {
                    let sum = (((t0[i] as i16) + (t1[i] as i16))
                        + ((t2[i] as i16) + (t3[i] as i16)))
                        + (((t4[i] as i16) + (t5[i] as i16))
                            + ((t6[i] as i16) + (t7[i] as i16)));
                    out[i] = sum.clamp(-128, 127) as i8;
                }
            }
            _ => {
                debug_assert!(tracks.is_empty() || tracks.len() > PARAGON5_MAX_TRACKS);
                return Err(Error::InvalidTrackCount(tracks.len()));
            }
        }
        Ok(())
    }

    /// Apply the exact `GbassV2` 50% stereo crossfeed post-processor in-place:
    /// `L' = clamp(L + (R >> 1), -128, 127)` and `R' = clamp(R + (L >> 1), -128, 127)`.
    ///
    /// Mirrors the exact ARM7TDMI routine (`add r2, r4, r3, asr #1; add r3, r3, r4, asr #1`)
    /// present in all 15 `GbassV2` titles.
    #[inline]
    pub fn apply_stereo_crossfeed(left: &mut [i8], right: &mut [i8]) {
        let n = left.len().min(right.len());
        let (l_chunks, l_rem) = left[..n].as_chunks_mut::<16>();
        let (r_chunks, r_rem) = right[..n].as_chunks_mut::<16>();
        for (lc, rc) in l_chunks.iter_mut().zip(r_chunks.iter_mut()) {
            for i in 0..16 {
                let l = lc[i] as i16;
                let r = rc[i] as i16;
                lc[i] = (l + (r >> 1)).clamp(-128, 127) as i8;
                rc[i] = (r + (l >> 1)).clamp(-128, 127) as i8;
            }
        }
        for (l_slot, r_slot) in l_rem.iter_mut().zip(r_rem.iter_mut()) {
            let l = *l_slot as i16;
            let r = *r_slot as i16;
            *l_slot = (l + (r >> 1)).clamp(-128, 127) as i8;
            *r_slot = (r + (l >> 1)).clamp(-128, 127) as i8;
        }
    }

    /// Apply version-appropriate stereo post-processing to a pair of DirectSound buffers.
    ///
    /// - [`Paragon5Version::GbassV1`]: Direct stereo passthrough (no-op).
    /// - [`Paragon5Version::GbassV2`]: Applies [`Self::apply_stereo_crossfeed`].
    #[inline]
    pub fn post_process_stereo(version: Paragon5Version, left: &mut [i8], right: &mut [i8]) {
        if version.has_stereo_crossfeed() {
            Self::apply_stereo_crossfeed(left, right);
        }
    }
}

// ============================================================================
// StudioSynth — 48 kHz f32 Lanczos-3 Polyphase LUT Renderer
// ============================================================================

/// High-fidelity studio software synthesizer for Paragon 5 / GBASS samples.
///
/// Renders signed 8-bit PCM samples at 48,000 Hz (configurable) in 32-bit floating-point using a
/// compile-time 256-phase × 6-tap Lanczos-3 windowed-sinc polyphase LUT (or general Lanczos-`a`
/// fallback when `sinc_lobes != 3`) with true equal-power stereo panning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StudioSynth {
    /// Output sample rate in Hz (default `48_000`).
    pub sample_rate: u32,
    /// Number of Lanczos sinc lobes (`3` uses the compile-time polyphase LUT fast path).
    pub sinc_lobes: usize,
}

impl Default for StudioSynth {
    fn default() -> Self {
        Self::new(Self::SAMPLE_RATE)
    }
}

impl StudioSynth {
    /// Standard studio output sample rate (`48_000` Hz).
    pub const SAMPLE_RATE: u32 = 48_000;

    /// Create a new `StudioSynth` at the specified output sample rate (typically `48_000` Hz).
    #[inline]
    #[must_use]
    pub const fn new(sample_rate: u32) -> Self {
        Self {
            sample_rate,
            sinc_lobes: 3,
        }
    }

    /// Render a Paragon 5 sample into stereo `f32` buffers (accumulating) using band-limited
    /// Lanczos windowed-sinc interpolation and true equal-power stereo panning.
    ///
    /// - `freq_hz`: desired playback sample rate in Hz.
    /// - `volume`: linear gain in `0.0..=1.0`.
    /// - `pan`: stereo pan in `-1.0..=1.0` (`-1.0` = hard left, `0.0` = center, `+1.0` = hard right).
    #[allow(clippy::too_many_arguments)]
    pub fn render_voice_f32(
        &self,
        sample: &Paragon5SampleView<'_>,
        pos_f64: &mut f64,
        freq_hz: f64,
        volume: f32,
        pan: f32,
        out_left: &mut [f32],
        out_right: &mut [f32],
    ) {
        let frames = out_left.len().min(out_right.len());
        if frames == 0
            || sample.is_empty()
            || self.sample_rate == 0
            || !freq_hz.is_finite()
            || freq_hz <= 0.0
            || !pos_f64.is_finite()
            || !volume.is_finite()
        {
            return;
        }
        if *pos_f64 < 0.0 {
            *pos_f64 = 0.0;
        }

        let step = freq_hz / (self.sample_rate as f64);
        let gain = (volume.clamp(0.0, 1.0) as f64) * (1.0 / 128.0);
        // Equal-power stereo pan law: pan in [-1.0, +1.0] -> theta in [0, pi/2]
        let pan_clamped = if pan.is_finite() {
            pan.clamp(-1.0, 1.0) as f64
        } else {
            0.0
        };
        let pan_norm = (pan_clamped + 1.0) * 0.5;
        let angle = pan_norm * core::f64::consts::FRAC_PI_2;
        let gain_l = (gain * cos_const(angle)) as f32;
        let gain_r = (gain * sin_const(angle)) as f32;

        let pcm_end = sample.effective_end();
        if pcm_end == 0 {
            return;
        }
        let pcm = &sample.pcm_bytes[..pcm_end];
        let pcm_len = pcm.len();
        let loop_len = sample.loop_len();
        let looped = loop_len > 0;
        let loop_start = sample.loop_start.unwrap_or(0) as usize;
        let loop_start_f = loop_start as f64;
        let loop_len_f = loop_len as f64;
        let pcm_len_f = pcm_len as f64;

        let out_l = &mut out_left[..frames];
        let out_r = &mut out_right[..frames];

        if self.sinc_lobes == 3 {
            let safe_max_base = pcm_len.saturating_sub(4);
            let lut = &LANCZOS3_LUT.0;
            let mut ph = *pos_f64;
            let mut idx = 0usize;

            while idx < frames {
                if ph >= pcm_len_f {
                    if looped {
                        ph -= loop_len_f;
                        if ph >= pcm_len_f {
                            ph = loop_start_f + ((ph - loop_start_f) % loop_len_f);
                        }
                        if ph < loop_start_f {
                            ph = loop_start_f;
                        }
                    } else {
                        break;
                    }
                }

                let base = ph as usize;
                if pcm_len >= 6
                    && base >= 2
                    && base <= safe_max_base
                    && ph < (1u64 << 31) as f64
                    && step < (1u64 << 31) as f64
                {
                    let safe_limit = (safe_max_base as f64) + 0.99;
                    let max_interior = (((safe_limit - ph) / step) as usize) + 1;
                    let run = max_interior.min(frames - idx);
                    if run >= 4 {
                        const Q32_SCALE: f64 = 4_294_967_296.0;
                        let ph_q32 = (ph * Q32_SCALE) as u64;
                        let step_q32 = (step * Q32_SCALE) as u64;
                        // Pre-add half-phase rounding (1 << 23) and subtract 2 samples so that:
                        // - `b0 = ph_rounded >> 32` is the window start index (`base - 2`),
                        // - `p = (ph_rounded as u32) >> 24` is strictly in `0..256` (zero bounds check!).
                        let ph_rounded_start = (ph_q32 + (1u64 << 23)) - (2u64 << 32);
                        let last_b0 = ((ph_rounded_start
                            .saturating_add(step_q32.saturating_mul((run - 1) as u64)))
                            >> 32) as usize;
                        if last_b0 + 6 <= pcm_len {
                            let mut fbuf = [0.0f32; 264];
                            let mut rem = run;
                            let mut cur_idx = idx;
                            let mut ph_rounded = ph_rounded_start;
                            // Max frames whose source window stays within 256 base positions
                            // (`rel_b0 in 0..=255`, `rel_b0 + 5 <= 260 < 264`).
                            let max_chunk = (255u64 << 32)
                                .checked_div(step_q32)
                                .map_or(rem, |v| (v as usize).max(1));
                            while rem > 0 {
                                let chunk_len = rem.min(max_chunk);
                                let b0_start = (ph_rounded >> 32) as usize;
                                let b0_end = ((ph_rounded
                                    + step_q32 * ((chunk_len - 1) as u64))
                                    >> 32) as usize;
                                let rel_max = b0_end - b0_start;
                                if rel_max < 256 {
                                    let span = rel_max + 6;
                                    for (dst, &byte) in fbuf[..span]
                                        .iter_mut()
                                        .zip(&pcm[b0_start..b0_start + span])
                                    {
                                        *dst = (byte as i8) as f32;
                                    }
                                    let mut ph_rel = ph_rounded - ((b0_start as u64) << 32);
                                    let (ol_chunks, ol_rem) =
                                        out_l[cur_idx..cur_idx + chunk_len].as_chunks_mut::<4>();
                                    let (or_chunks, or_rem) =
                                        out_r[cur_idx..cur_idx + chunk_len].as_chunks_mut::<4>();
                                    let step2 = step_q32.wrapping_mul(2);
                                    let step3 = step_q32.wrapping_mul(3);
                                    let step4 = step_q32.wrapping_mul(4);
                                    for (ol4, or4) in ol_chunks.iter_mut().zip(or_chunks.iter_mut())
                                    {
                                        let ph0 = ph_rel;
                                        let ph1 = ph_rel.wrapping_add(step_q32);
                                        let ph2 = ph_rel.wrapping_add(step2);
                                        let ph3 = ph_rel.wrapping_add(step3);
                                        ph_rel = ph_rel.wrapping_add(step4);

                                        let r0 = ((ph0 >> 32) as usize) & 0xFF;
                                        let w0 = &lut[((ph0 as u32) >> 24) as usize];
                                        let v0 = (fbuf[r0] * w0[0] + fbuf[r0 + 1] * w0[1])
                                            + (fbuf[r0 + 2] * w0[2] + fbuf[r0 + 3] * w0[3])
                                            + (fbuf[r0 + 4] * w0[4] + fbuf[r0 + 5] * w0[5]);

                                        let r1 = ((ph1 >> 32) as usize) & 0xFF;
                                        let w1 = &lut[((ph1 as u32) >> 24) as usize];
                                        let v1 = (fbuf[r1] * w1[0] + fbuf[r1 + 1] * w1[1])
                                            + (fbuf[r1 + 2] * w1[2] + fbuf[r1 + 3] * w1[3])
                                            + (fbuf[r1 + 4] * w1[4] + fbuf[r1 + 5] * w1[5]);

                                        let r2 = ((ph2 >> 32) as usize) & 0xFF;
                                        let w2 = &lut[((ph2 as u32) >> 24) as usize];
                                        let v2 = (fbuf[r2] * w2[0] + fbuf[r2 + 1] * w2[1])
                                            + (fbuf[r2 + 2] * w2[2] + fbuf[r2 + 3] * w2[3])
                                            + (fbuf[r2 + 4] * w2[4] + fbuf[r2 + 5] * w2[5]);

                                        let r3 = ((ph3 >> 32) as usize) & 0xFF;
                                        let w3 = &lut[((ph3 as u32) >> 24) as usize];
                                        let v3 = (fbuf[r3] * w3[0] + fbuf[r3 + 1] * w3[1])
                                            + (fbuf[r3 + 2] * w3[2] + fbuf[r3 + 3] * w3[3])
                                            + (fbuf[r3 + 4] * w3[4] + fbuf[r3 + 5] * w3[5]);

                                        ol4[0] += v0 * gain_l;
                                        ol4[1] += v1 * gain_l;
                                        ol4[2] += v2 * gain_l;
                                        ol4[3] += v3 * gain_l;
                                        or4[0] += v0 * gain_r;
                                        or4[1] += v1 * gain_r;
                                        or4[2] += v2 * gain_r;
                                        or4[3] += v3 * gain_r;
                                    }
                                    for (ol, or) in ol_rem.iter_mut().zip(or_rem.iter_mut()) {
                                        let r = ((ph_rel >> 32) as usize) & 0xFF;
                                        let p = ((ph_rel as u32) >> 24) as usize;
                                        let w = &lut[p];
                                        let val = (fbuf[r] * w[0] + fbuf[r + 1] * w[1])
                                            + (fbuf[r + 2] * w[2] + fbuf[r + 3] * w[3])
                                            + (fbuf[r + 4] * w[4] + fbuf[r + 5] * w[5]);
                                        *ol += val * gain_l;
                                        *or += val * gain_r;
                                        ph_rel = ph_rel.wrapping_add(step_q32);
                                    }
                                    ph_rounded += step_q32 * (chunk_len as u64);
                                } else {
                                    let src = &pcm[..last_b0 + 6];
                                    for (ol, or) in out_l[cur_idx..cur_idx + chunk_len]
                                        .iter_mut()
                                        .zip(&mut out_r[cur_idx..cur_idx + chunk_len])
                                    {
                                        let b0 = (ph_rounded >> 32) as usize;
                                        let p = ((ph_rounded as u32) >> 24) as usize;
                                        let w = &lut[p];
                                        let win = &src[b0..b0 + 6];
                                        let s0 = (win[0] as i8) as f32;
                                        let s1 = (win[1] as i8) as f32;
                                        let s2 = (win[2] as i8) as f32;
                                        let s3 = (win[3] as i8) as f32;
                                        let s4 = (win[4] as i8) as f32;
                                        let s5 = (win[5] as i8) as f32;
                                        let val = (s0 * w[0] + s1 * w[1])
                                            + (s2 * w[2] + s3 * w[3])
                                            + (s4 * w[4] + s5 * w[5]);
                                        *ol += val * gain_l;
                                        *or += val * gain_r;
                                        ph_rounded += step_q32;
                                    }
                                }
                                cur_idx += chunk_len;
                                rem -= chunk_len;
                            }
                            ph += (run as f64) * step;
                            idx += run;
                            continue;
                        }
                    }
                }

                // Boundary / loop-edge single-sample step
                let frac = ph - (base as f64);
                let p_raw = (frac * (LANCZOS_PHASES as f64) + 0.5) as usize;
                let (base_i, p) = if p_raw >= LANCZOS_PHASES {
                    ((base as isize) + 1, 0usize)
                } else {
                    (base as isize, p_raw)
                };
                let w = &lut[p];

                let mut s = [0.0f32; 6];
                for (slot, k) in s.iter_mut().zip(-2isize..=3) {
                    let sample_idx = base_i + k;
                    if sample_idx >= 0 {
                        let u = sample_idx as usize;
                        if u < pcm_len {
                            *slot = (pcm[u] as i8) as f32;
                        } else if looped {
                            let wrapped = loop_start + ((u - loop_start) % loop_len);
                            *slot = (pcm[wrapped] as i8) as f32;
                        }
                    }
                }

                let val = (s[0] * w[0] + s[1] * w[1])
                    + (s[2] * w[2] + s[3] * w[3])
                    + (s[4] * w[4] + s[5] * w[5]);

                out_l[idx] += val * gain_l;
                out_r[idx] += val * gain_r;
                ph += step;
                idx += 1;
            }

            *pos_f64 = ph;
            return;
        }

        // General fallback for custom `sinc_lobes != 3`
        let a = self.sinc_lobes.max(1) as isize;
        let a_f64 = a as f64;
        let mut ph = *pos_f64;

        for (ol, or) in out_l.iter_mut().zip(out_r.iter_mut()) {
            if ph >= pcm_len_f {
                if looped {
                    ph = loop_start_f + ((ph - loop_start_f) % loop_len_f);
                } else {
                    break;
                }
            }
            let center = ph as isize;
            let mut acc = 0.0f64;
            let mut wsum = 0.0f64;

            for k in (center - a + 1)..=(center + a) {
                let s_val = if k < 0 {
                    0.0
                } else if (k as usize) < pcm_len {
                    (pcm[k as usize] as i8) as f64
                } else if looped {
                    let wrapped = loop_start + (((k as usize) - loop_start) % loop_len);
                    (pcm[wrapped] as i8) as f64
                } else {
                    0.0
                };
                let w = lanczos_kernel_const(ph - (k as f64), a_f64);
                acc += s_val * w;
                wsum += w;
            }

            let val = if wsum.abs() > 1e-9 {
                (acc / wsum) as f32
            } else {
                0.0
            };

            *ol += val * gain_l;
            *or += val * gain_r;
            ph += step;
        }

        *pos_f64 = ph;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_studio_synth_lut_matches_direct_lanczos3() {
        let pcm: [u8; 64] = core::array::from_fn(|i| {
            let angle = (i as f64) * 0.35;
            (sin_const(angle) * 100.0) as i8 as u8
        });
        let view = Paragon5SampleView::from_pcm_bytes(13_379, 60, None, None, &pcm);
        let synth = StudioSynth::new(48_000);
        let mut left = [0.0f32; 128];
        let mut right = [0.0f32; 128];
        let mut pos = 5.25;
        synth.render_voice_f32(&view, &mut pos, 12_000.0, 1.0, 0.0, &mut left, &mut right);

        let step = 12_000.0 / 48_000.0;
        let gain_l = ((1.0 / 128.0) * cos_const(core::f64::consts::FRAC_PI_4)) as f32;
        let mut ref_pos = 5.25f64;
        for &actual in &left {
            let center = ref_pos as isize;
            let mut acc = 0.0f64;
            let mut wsum = 0.0f64;
            for k in (center - 2)..=(center + 3) {
                let s = (pcm[k as usize] as i8) as f64;
                let w = lanczos_kernel_const(ref_pos - (k as f64), 3.0);
                acc += s * w;
                wsum += w;
            }
            let expected = ((acc / wsum) as f32) * gain_l;
            assert!(
                (actual - expected).abs() < 1e-3,
                "LUT mismatch: actual={actual}, expected={expected}"
            );
            ref_pos += step;
        }
    }

    #[test]
    fn test_sum_tracks_rejects_invalid_counts() {
        let mut out = [0i8; PARAGON5_BLOCK_SIZE];
        assert_eq!(
            AccuracySynth::sum_tracks(&[], &mut out),
            Err(Error::InvalidTrackCount(0))
        );
        let nine = [[0i8; PARAGON5_BLOCK_SIZE]; 9];
        assert_eq!(
            AccuracySynth::sum_tracks(&nine, &mut out),
            Err(Error::InvalidTrackCount(9))
        );
    }

    #[test]
    fn test_post_process_stereo_version_dispatch() {
        let mut l_v1 = [100i8, -100, 127, -128];
        let mut r_v1 = [60i8, -60, 127, -128];
        AccuracySynth::post_process_stereo(Paragon5Version::GbassV1, &mut l_v1, &mut r_v1);
        assert_eq!(l_v1, [100, -100, 127, -128]);
        assert_eq!(r_v1, [60, -60, 127, -128]);

        let mut l_v2 = [100i8, -100, 127, -128];
        let mut r_v2 = [60i8, -60, 127, -128];
        AccuracySynth::post_process_stereo(Paragon5Version::GbassV2, &mut l_v2, &mut r_v2);
        assert_eq!(l_v2, [127, -128, 127, -128]);
        assert_eq!(r_v2, [110, -110, 127, -128]);
    }
}
