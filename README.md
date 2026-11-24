# paragon5-rs

**Zero-dependency, pure Rust parser, software synthesizer, and byte-exact golden-tested emulator for the Paragon 5 / GBASS GBA Sound Engine (Game Boy Advance).**

[![Crates.io](https://img.shields.io/crates/v/paragon5.svg)](https://crates.io/crates/paragon5)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE-MIT)
[![no_std](https://img.shields.io/badge/no__std-compatible-success.svg)](#features)

The **Paragon 5 GBASS Sound System** powered **17 commercial and prototype Game Boy Advance releases** including *Magi Nation* (JP), *Godzilla: Domination!* (US/EU/JP/Proto), *Rugrats: I Gotta Go Party* (US/FR/3-in-1), *The Scorpion King: Sword of Osiris* (US/EU), *Micro Machines* (EU), *David Beckham Soccer* (US/EU), *Hardcore Pinball*, *X-Bladez: Inline Skater* (US/EU), and *Karnaaj Rally*.

`paragon5` provides a clean-room, reverse-engineered implementation designed for audio preservation, DAWs, PC source ports, and emulators.

> **Looking for a ready-to-use CLI?** You can scan GBA ROMs, extract WAV samples, and compile SoundFont 2.04 (.sf2) banks directly from the command line using our **[gbakit](https://github.com/JidouGhost-TL/gbakit)** toolkit (`gbakit audio scan`, `gbakit audio export`, `gbakit audio soundfont`).

---

## Features

- **Zero External Dependencies**: Pure Rust with no dependencies in `[dependencies]`, `#![deny(unsafe_code)]`.
- **`#![no_std]` Compatible**: Ready for embedded targets and WASM runtimes (`alloc` optional).
- **Native Binary Parsers**: Zero-allocation parsers for the 64-byte master audio header (`Paragon5MasterHeader`) and 24-byte 16.16 fixed-point PCM8 sample descriptor (`Paragon5SampleDesc`).
- **Dual Synthesis Engines**:
  - **`AccuracySynth`**: 100% bit-exact emulation of both Paragon 5 / GBASS ARM7TDMI mixer revisions — including 16.16 fixed-point dual-voice pairing with 3-stage volume scaling, signed `smull >> 16` pitch scaling, one-shot freeze-at-end (`loop_len == base_step`), 1..=8-track saturating `[-128, +127]` summing, and the `GbassV2` 50% stereo crossfeed post-processor.
  - **`StudioSynth`**: Renders at 48,000 Hz, 32-bit floating-point precision with a compile-time 1024-phase × 6-tap Lanczos-3 polyphase windowed-sinc LUT (plus general Lanczos-`a` fallback) and true equal-power stereo panning.
- **Byte-Exact `libmgba` ARM Golden Tested**: 1,024 / 1,024 output bytes + 12 updated 16.16 voice positions verified strictly byte-for-byte (`assert_eq!`) across both GBASS revisions.

---

## Supported Paragon 5 / GBASS Engine Revisions

Auditing all 3,377 GBA ROMs identifies **2 distinct ARM7TDMI mixer revisions** spanning **17 commercial/prototype titles**, both detected via `Paragon5Version::detect_from_bytes` and byte-exact golden-tested:

| Revision | Corpus Share | Representative Titles | Voice-Pair Scratch & Stepping | Summing Mixer | Stereo Post-Processing |
| --- | ---: | --- | --- | --- | --- |
| `GbassV1` | 2 ROMs | *David Beckham Soccer* (USA, Europe) | Direct `sp` save to IWRAM; 16.16 FP, `smull >> 16`, `(sa*va + sb*vb) >> 9` | 1..=8 tracks, saturating `[-128, +127]` | Direct stereo (no crossfeed) |
| `GbassV2` | 15 ROMs | *Magi Nation*, *Godzilla: Domination!* (US/EU/JP/Proto), *Rugrats* (US/FR/3-in-1), *Scorpion King*, *Micro Machines*, *Hardcore Pinball*, *X-Bladez*, *Karnaaj Rally* | ARM UNDEFINED-mode (`CPSR = 0x1B`) `sp_und` scratch; identical 16.16 FP math | 1..=8 tracks, saturating `[-128, +127]` | 50% stereo crossfeed: `L' = clamp(L + (R >> 1))`, `R' = clamp(R + (L >> 1))` |

---

## Quickstart

### Zero-Allocation Mixing (`#![no_std]`)

```rust
use paragon5::{AccuracySynth, Paragon5Version, Paragon5Voice, PARAGON5_BLOCK_SIZE};

// Raw signed 8-bit PCM waveforms (0 = silence)
let pcm_a: &[u8] = &[0, 32, 64, 96, 127, 96, 64, 32, 0, 224, 192, 160, 128, 160, 192, 224];
let pcm_b: &[u8] = &[0, 48, 96, 48, 0, 208, 160, 208, 0, 48, 96, 48, 0, 208, 160, 208];

let mut voice_a = Paragon5Voice::new_looped(
    0x0180, 0, 16 << 16, 0x0001_0000, 16 << 16, 0x0100, 0x0100, 0x0001_0000,
);
let mut voice_b = Paragon5Voice::new_oneshot(
    0x0100, 0, 16 << 16, 0x0000_8000, 0x0100, 0x0100, 0x0001_0000,
);

let mut track_l = [0i8; PARAGON5_BLOCK_SIZE];
let mut dma_l = [0i8; PARAGON5_BLOCK_SIZE];
let mut dma_r = [0i8; PARAGON5_BLOCK_SIZE];

AccuracySynth::mix_voice_pair(pcm_a, &mut voice_a, pcm_b, &mut voice_b, &mut track_l);
AccuracySynth::sum_tracks(&[track_l], &mut dma_l).unwrap();
AccuracySynth::post_process_stereo(Paragon5Version::GbassV2, &mut dma_l, &mut dma_r);
```

---

## Golden Test Methodology

Every mixing path in `AccuracySynth` is verified strictly byte-for-byte (`assert_eq!`, zero tolerance) against real ARM7TDMI execution traces captured in headless `libmgba.so.0.10` using clean-room synthetic deterministic waveforms:

- **6 GBASS v1 ARM vectors (384 bytes + 6 final `pos_fp16`)**: Looped fractional + `pitch_scale_fp16` + 3-stage volume, one-shot freeze-at-end (`loop_len == base_step`) + loop, fast pitch boundary, and 1-, 4-, and 8-track saturating sums.
- **7 GBASS v2 ARM vectors (640 bytes + 6 final `pos_fp16`)**: All 3 voice-pair vectors under ARM UNDEFINED-mode (`CPSR = 0x1B`), 1-, 6-, and 8-track saturating sums, and 128-frame (256-byte) 50% stereo crossfeed post-processing.

---

## License

Licensed under either of:
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))
