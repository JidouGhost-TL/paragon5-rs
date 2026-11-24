//! # paragon5-rs
//!
//! Pure Rust, zero-dependency parser, software synthesizer, and byte-exact golden-tested
//! emulator for the **Paragon 5 / GBASS GBA Sound Engine** on the Game Boy Advance.
//!
//! Covers all **17 commercial/prototype GBA releases** across both **2 distinct ARM7TDMI mixer
//! revisions** discovered in a full 3,377-ROM corpus audit:
//! - [`Paragon5Version::GbassV1`] (2 ROMs: *David Beckham Soccer* USA & Europe): Direct `sp`-save
//!   16.16 fixed-point voice-pair mixer, 1..=8-track saturating summing mixer, direct stereo output.
//! - [`Paragon5Version::GbassV2`] (15 ROMs: *Magi Nation*, *Godzilla: Domination!* US/EU/JP/Proto,
//!   *Rugrats: I Gotta Go Party* US/FR/3-in-1, *The Scorpion King: Sword of Osiris* US/EU,
//!   *Micro Machines*, *Hardcore Pinball*, *X-Bladez: Inline Skater* US/EU, *Karnaaj Rally*):
//!   ARM UNDEFINED-mode (`CPSR = 0x1B`) voice-pair mixer, 1..=8-track saturating summing mixer,
//!   and 50% stereo crossfeed post-processor (`L' = clamp(L + (R >> 1))`, `R' = clamp(R + (L >> 1))`).
//!
//! ## Features
//! - **Zero dependencies** in `[dependencies]`, `#![no_std]` compatible, `#![deny(unsafe_code)]`.
//! - **Dual synthesis pipelines**:
//!   - [`AccuracySynth`]: 100% bit-exact emulation of both Paragon 5 / GBASS ARM7TDMI revisions.
//!   - [`StudioSynth`]: 48,000 Hz `f32` band-limited Lanczos-3 polyphase LUT renderer with
//!     true equal-power stereo panning.
//! - **Byte-exact `libmgba` ARM golden tested**: 1,024 / 1,024 output bytes + 12 updated 16.16
//!   voice positions verified strictly byte-for-byte (`assert_eq!`) across both GBASS revisions.
//!
//! ## Quickstart
//!
//! ```rust
//! use paragon5::{AccuracySynth, Paragon5Version, Paragon5Voice, PARAGON5_BLOCK_SIZE};
//!
//! // Raw signed 8-bit PCM waveforms (0 = silence)
//! let pcm_a: &[u8] = &[0, 32, 64, 96, 127, 96, 64, 32, 0, 224, 192, 160, 128, 160, 192, 224];
//! let pcm_b: &[u8] = &[0, 48, 96, 48, 0, 208, 160, 208, 0, 48, 96, 48, 0, 208, 160, 208];
//!
//! let mut voice_a = Paragon5Voice::new_looped(
//!     0x0180, 0, 16 << 16, 0x0001_0000, 16 << 16, 0x0100, 0x0100, 0x0001_0000,
//! );
//! let mut voice_b = Paragon5Voice::new_oneshot(
//!     0x0100, 0, 16 << 16, 0x0000_8000, 0x0100, 0x0100, 0x0001_0000,
//! );
//!
//! let mut track_l = [0i8; PARAGON5_BLOCK_SIZE];
//! let mut dma_l = [0i8; PARAGON5_BLOCK_SIZE];
//! let mut dma_r = [0i8; PARAGON5_BLOCK_SIZE];
//!
//! AccuracySynth::mix_voice_pair(pcm_a, &mut voice_a, pcm_b, &mut voice_b, &mut track_l);
//! AccuracySynth::sum_tracks(&[track_l], &mut dma_l).unwrap();
//! AccuracySynth::post_process_stereo(Paragon5Version::GbassV2, &mut dma_l, &mut dma_r);
//! assert!(dma_l.iter().any(|&s| s != 0));
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod error;
pub mod freq;
pub mod sample;
pub mod synth;
pub mod version;
pub mod voice;

pub use error::{Error, Result};
pub use freq::{
    effective_step_q16, effective_volume, freq_to_step_q16, rate_from_timer_reload,
    step_q16_to_freq, GBA_CLOCK_HZ, PARAGON5_BLOCK_SIZE, PARAGON5_DEFAULT_MIX_RATE_HZ,
    PARAGON5_MAX_TRACKS, Q16_ONE,
};
#[cfg(feature = "alloc")]
pub use sample::Paragon5Sample;
pub use sample::{
    Paragon5MasterHeader, Paragon5SampleDesc, Paragon5SampleView, GBA_ROM_BUS_BASE,
    PARAGON5_MASTER_HEADER_SIZE, PARAGON5_PITCH_SCALE_ONE, PARAGON5_SAMPLE_DESC_SIZE,
};
pub use synth::{AccuracySynth, StudioSynth};
pub use version::Paragon5Version;
pub use voice::Paragon5Voice;
