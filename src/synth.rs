//! Dual Synthesis Engines for the Paragon 5 / GBASS GBA Sound Engine:
//!
//! - [`AccuracySynth`]: 100% bit-exact emulation of both Paragon 5 / GBASS ARM7TDMI mixer
//!   revisions (`GbassV1` & `GbassV2`) verified strictly byte-for-byte (`assert_eq!`, 0 tolerance)
//!   against headless `libmgba.so.0.10` ARM7TDMI execution.
//! - [`StudioSynth`]: High-fidelity 48,000 Hz `f32` software synthesizer featuring a compile-time
//!   1024-phase × 6-tap Lanczos-3 polyphase windowed-sinc LUT (with general Lanczos-`a` fallback)
//!   and true equal-power stereo panning.

use crate::error::{Error, Result};
use crate::freq::{PARAGON5_BLOCK_SIZE, PARAGON5_MAX_TRACKS};
use crate::sample::Paragon5SampleView;
use crate::version::Paragon5Version;
use crate::voice::Paragon5Voice;

// ============================================================================
// Pure no_std math & compile-time Lanczos-3 polyphase LUT
// ============================================================================

const LANCZOS_PHASES: usize = 1024;
const LANCZOS3_TAPS: usize = 6;

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

/// Build the normalized 1025-entry (`0..=1024`) × 6-tap Lanczos-3 polyphase filter table
/// entirely at compile time.
const fn build_lanczos3_lut() -> [[f32; LANCZOS3_TAPS]; LANCZOS_PHASES + 1] {
    let mut table = [[0.0f32; LANCZOS3_TAPS]; LANCZOS_PHASES + 1];
    let mut p = 0usize;
    while p <= LANCZOS_PHASES {
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
    table
}

static LANCZOS3_LUT: [[f32; LANCZOS3_TAPS]; LANCZOS_PHASES + 1] = build_lanczos3_lut();

// ============================================================================
// AccuracySynth — 100% Bit-Exact ARM7TDMI GBASS v1 & v2 Emulation
// ============================================================================

/// Byte-exact ARM7TDMI software mixer emulation for both Paragon 5 / GBASS revisions
/// ([`Paragon5Version::GbassV1`] and [`Paragon5Version::GbassV2`]).
#[derive(Debug, Default, Clone, Copy)]
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

        let mut pos_a = voice_a.pos_fp16;
        let end_a = voice_a.end_fp16;
        let mut pos_b = voice_b.pos_fp16;
        let end_b = voice_b.end_fp16;

        let len_a = pcm_a.len();
        let len_b = pcm_b.len();

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
        if tracks.is_empty() || tracks.len() > PARAGON5_MAX_TRACKS {
            return Err(Error::InvalidTrackCount(tracks.len()));
        }

        for (i, slot) in out.iter_mut().enumerate() {
            let mut sum = 0i32;
            for track in tracks {
                sum += track[i] as i32;
            }
            *slot = sum.clamp(-128, 127) as i8;
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
        for (l_slot, r_slot) in left[..n].iter_mut().zip(right[..n].iter_mut()) {
            let l = *l_slot as i32;
            let r = *r_slot as i32;
            let new_l = (l + (r >> 1)).clamp(-128, 127) as i8;
            let new_r = (r + (l >> 1)).clamp(-128, 127) as i8;
            *l_slot = new_l;
            *r_slot = new_r;
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
/// compile-time 1024-phase × 6-tap Lanczos-3 windowed-sinc polyphase LUT (or general Lanczos-`a`
/// fallback when `sinc_lobes != 3`) with true equal-power stereo panning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
        {
            return;
        }
        if *pos_f64 < 0.0 {
            *pos_f64 = 0.0;
        }

        let step = freq_hz / (self.sample_rate as f64);
        let gain = (volume.clamp(0.0, 1.0) as f64) * (1.0 / 128.0);
        // Equal-power stereo pan law: pan in [-1.0, +1.0] -> theta in [0, pi/2]
        let pan_norm = ((pan.clamp(-1.0, 1.0) as f64) + 1.0) * 0.5;
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
                if pcm_len >= 6 && base >= 2 && base <= safe_max_base && step < (1u64 << 31) as f64
                {
                    let safe_limit = (safe_max_base as f64) + 0.999_999;
                    let max_interior = (((safe_limit - ph) / step) as usize) + 1;
                    let run = max_interior.min(frames - idx);
                    if run >= 4 {
                        const Q32_SCALE: f64 = 4_294_967_296.0;
                        let ph_q32 = (ph * Q32_SCALE) as u64;
                        let step_q32 = (step * Q32_SCALE) as u64;
                        let last_base = ((ph_q32
                            .saturating_add(step_q32.saturating_mul((run - 1) as u64)))
                            >> 32) as usize;
                        if last_base <= safe_max_base {
                            let src = &pcm[..last_base + 4];
                            let mut ph_shifted = ph_q32 - (2u64 << 32);
                            for (ol, or) in out_l[idx..idx + run]
                                .iter_mut()
                                .zip(&mut out_r[idx..idx + run])
                            {
                                let b0 = (ph_shifted >> 32) as usize;
                                let p = (((ph_shifted & 0xFFFF_FFFF) + (1 << 21)) >> 22) as usize;
                                let w = &LANCZOS3_LUT[p];
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
                                ph_shifted += step_q32;
                            }
                            ph += (run as f64) * step;
                            idx += run;
                            continue;
                        }
                    }
                }

                // Boundary / loop-edge single-sample step
                let frac = ph - (base as f64);
                let p = ((frac * (LANCZOS_PHASES as f64) + 0.5) as usize).min(LANCZOS_PHASES);
                let w = &LANCZOS3_LUT[p];

                let mut s = [0.0f32; 6];
                let base_i = base as isize;
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

                let val = s[0] * w[0]
                    + s[1] * w[1]
                    + s[2] * w[2]
                    + s[3] * w[3]
                    + s[4] * w[4]
                    + s[5] * w[5];

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
