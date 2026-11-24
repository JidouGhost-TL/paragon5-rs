//! Byte-exact ARM7TDMI golden tests for Paragon 5 / GBASS v1 & v2.
//!
//! Every golden `.bin` vector in `tests/golden/` was captured by executing the real ARM7TDMI
//! GBASS v1 and GBASS v2 mixer routines in headless `libmgba.so.0.10` using clean-room synthetic
//! deterministic waveforms (zero copyrighted game audio data). Every single output byte and
//! updated 16.16 fixed-point voice position is verified with strict `assert_eq!` (0 tolerance).

use paragon5::{AccuracySynth, Paragon5Version, Paragon5Voice, PARAGON5_BLOCK_SIZE};

fn make_wave_a() -> [u8; 256] {
    core::array::from_fn(|i| (((i as i32 * 17 + 23) % 256) - 128) as i8 as u8)
}

fn make_wave_b() -> [u8; 256] {
    core::array::from_fn(|i| {
        let v = ((i * 29) ^ (i * 7)) & 0xFF;
        let sv = (((v as i32) - 128) * 15) >> 4;
        sv as i8 as u8
    })
}

fn make_tracks_8() -> [[i8; PARAGON5_BLOCK_SIZE]; 8] {
    core::array::from_fn(|t| {
        core::array::from_fn(|i| {
            if i < 8 {
                if i % 2 == 0 { 40 } else { -40 }
            } else if i < 16 {
                ((i as i32 - 12) * 15) as i8
            } else {
                (((i * (t + 5) * 13 + t * 37) % 256) as u8) as i8
            }
        })
    })
}

fn make_crossfeed_inputs() -> ([i8; 128], [i8; 128]) {
    let edge_pairs: [(i8, i8); 16] = [
        (0, 0),
        (-1, -1),
        (-3, 1),
        (127, 127),
        (-128, -128),
        (100, 60),
        (-100, -60),
        (127, -128),
        (-128, 127),
        (85, 86),
        (-85, -86),
        (64, 127),
        (-64, -128),
        (1, -1),
        (-2, -3),
        (126, 3),
    ];
    let mut left = [0i8; 128];
    let mut right = [0i8; 128];
    for i in 0..128 {
        if i < 16 {
            left[i] = edge_pairs[i].0;
            right[i] = edge_pairs[i].1;
        } else {
            left[i] = ((i * 19 + 5) & 0xFF) as u8 as i8;
            right[i] = ((i * 31 + 101) & 0xFF) as u8 as i8;
        }
    }
    (left, right)
}

fn assert_matches_golden_i8(actual: &[i8], golden_bytes: &[u8], label: &str) {
    assert_eq!(actual.len(), golden_bytes.len(), "{label}: length mismatch");
    for (idx, (&a, &g_u8)) in actual.iter().zip(golden_bytes.iter()).enumerate() {
        let g = g_u8 as i8;
        assert_eq!(a, g, "{label}: byte mismatch at index {idx}");
    }
}

#[test]
fn golden_voice_pair_looped_fractional_v1_and_v2() {
    let wave_a = make_wave_a();
    let wave_b = make_wave_b();

    let mut va = Paragon5Voice::new_looped(
        0x0180,
        0x0002_8000,
        0x0040_0000,
        0x0001_4000,
        0x0020_0000,
        0x0100,
        0x00C0,
        0x0001_2000,
    );
    let mut vb = Paragon5Voice::new_looped(
        0x0120,
        0x0010_4000,
        0x0050_0000,
        0x0000_C000,
        0x0030_0000,
        0x0140,
        0x0080,
        0x0000_E000,
    );

    let mut out = [0i8; PARAGON5_BLOCK_SIZE];
    AccuracySynth::mix_voice_pair(&wave_a, &mut va, &wave_b, &mut vb, &mut out);

    assert_matches_golden_i8(
        &out,
        include_bytes!("golden/v1_vp_looped_fractional.bin"),
        "v1_vp_looped_fractional",
    );
    assert_matches_golden_i8(
        &out,
        include_bytes!("golden/v2_vp_looped_fractional.bin"),
        "v2_vp_looped_fractional",
    );
    assert_eq!(va.pos_fp16, 0x003C_8000);
    assert_eq!(vb.pos_fp16, 0x003A_4000);
}

#[test]
fn golden_voice_pair_oneshot_freeze_and_loop_v1_and_v2() {
    let wave_a = make_wave_a();
    let wave_b = make_wave_b();

    let mut va = Paragon5Voice::new_oneshot(
        0x0200,
        0x001C_0000,
        0x0020_0000,
        0x0000_8000,
        0x0100,
        0x0100,
        0x0001_8000,
    );
    let mut vb = Paragon5Voice::new_looped(
        0x01C0,
        0x0000_0000,
        0x0030_0000,
        0x0001_A000,
        0x0018_0000,
        0x0100,
        0x0100,
        0x0001_0000,
    );

    let mut out = [0i8; PARAGON5_BLOCK_SIZE];
    AccuracySynth::mix_voice_pair(&wave_a, &mut va, &wave_b, &mut vb, &mut out);

    assert_matches_golden_i8(
        &out,
        include_bytes!("golden/v1_vp_oneshot_freeze_and_loop.bin"),
        "v1_vp_oneshot_freeze_and_loop",
    );
    assert_matches_golden_i8(
        &out,
        include_bytes!("golden/v2_vp_oneshot_freeze_and_loop.bin"),
        "v2_vp_oneshot_freeze_and_loop",
    );
    assert_eq!(va.pos_fp16, 0x001F_C000);
    assert_eq!(vb.pos_fp16, 0x0020_0000);
}

#[test]
fn golden_voice_pair_fast_pitch_boundary_v1_and_v2() {
    let wave_a = make_wave_a();
    let wave_b = make_wave_b();

    let mut va = Paragon5Voice::new_looped(
        0x0280,
        0x0000_0000,
        0x0080_0000,
        0x0002_4000,
        0x0040_0000,
        0x0180,
        0x00A0,
        0x0001_1000,
    );
    let mut vb = Paragon5Voice::new_oneshot(
        0x00E0,
        0x0005_0000,
        0x0010_0000,
        0x0000_4000,
        0x00E0,
        0x00F0,
        0x0001_0000,
    );

    let mut out = [0i8; PARAGON5_BLOCK_SIZE];
    AccuracySynth::mix_voice_pair(&wave_a, &mut va, &wave_b, &mut vb, &mut out);

    assert_matches_golden_i8(
        &out,
        include_bytes!("golden/v1_vp_fast_pitch_boundary.bin"),
        "v1_vp_fast_pitch_boundary",
    );
    assert_matches_golden_i8(
        &out,
        include_bytes!("golden/v2_vp_fast_pitch_boundary.bin"),
        "v2_vp_fast_pitch_boundary",
    );
    assert_eq!(va.pos_fp16, 0x0059_0000);
    assert_eq!(vb.pos_fp16, 0x000F_C000);
}

#[test]
fn golden_sum_tracks_v1_and_v2() {
    let tracks = make_tracks_8();
    let mut out = [0i8; PARAGON5_BLOCK_SIZE];

    // 1 track (v1 & v2)
    AccuracySynth::sum_tracks(&tracks[..1], &mut out).expect("sum 1");
    assert_matches_golden_i8(&out, include_bytes!("golden/v1_sum_1.bin"), "v1_sum_1");
    assert_matches_golden_i8(&out, include_bytes!("golden/v2_sum_1.bin"), "v2_sum_1");

    // 4 tracks (v1)
    AccuracySynth::sum_tracks(&tracks[..4], &mut out).expect("sum 4");
    assert_matches_golden_i8(&out, include_bytes!("golden/v1_sum_4.bin"), "v1_sum_4");

    // 6 tracks (v2)
    AccuracySynth::sum_tracks(&tracks[..6], &mut out).expect("sum 6");
    assert_matches_golden_i8(&out, include_bytes!("golden/v2_sum_6.bin"), "v2_sum_6");

    // 8 tracks (v1 & v2)
    AccuracySynth::sum_tracks(&tracks[..8], &mut out).expect("sum 8");
    assert_matches_golden_i8(&out, include_bytes!("golden/v1_sum_8.bin"), "v1_sum_8");
    assert_matches_golden_i8(&out, include_bytes!("golden/v2_sum_8.bin"), "v2_sum_8");
}

#[test]
fn golden_gbass_v2_stereo_crossfeed_128() {
    let (mut left, mut right) = make_crossfeed_inputs();
    let orig_left = left;
    let orig_right = right;

    // GBASS v1 must leave stereo untouched
    AccuracySynth::post_process_stereo(Paragon5Version::GbassV1, &mut left, &mut right);
    assert_eq!(left, orig_left);
    assert_eq!(right, orig_right);

    // GBASS v2 applies 50% stereo crossfeed
    AccuracySynth::post_process_stereo(Paragon5Version::GbassV2, &mut left, &mut right);
    assert_matches_golden_i8(
        &left,
        include_bytes!("golden/v2_crossfeed_128_l.bin"),
        "v2_crossfeed_128_l",
    );
    assert_matches_golden_i8(
        &right,
        include_bytes!("golden/v2_crossfeed_128_r.bin"),
        "v2_crossfeed_128_r",
    );
}
