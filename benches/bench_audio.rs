use std::hint::black_box;
use std::time::Instant;

use paragon5::{
    AccuracySynth, Paragon5SampleView, Paragon5Version, Paragon5Voice, StudioSynth,
    PARAGON5_BLOCK_SIZE,
};

fn main() {
    println!("=== paragon5-rs Audio Performance Benchmarks ===");

    let pcm_a: Vec<u8> = (0i32..4096)
        .map(|i| (((i * 19 + 7) % 256) - 128) as i8 as u8)
        .collect();
    let pcm_b: Vec<u8> = (0i32..4096)
        .map(|i| (((i * 31 + 53) % 256) - 128) as i8 as u8)
        .collect();
    let sample = Paragon5SampleView::from_pcm_bytes(13_379, 60, Some(512), Some(4096), &pcm_a);

    let iters = 500_000usize;

    // 1. AccuracySynth::mix_voice_pair (64 frames)
    let mut track_buf = [0i8; PARAGON5_BLOCK_SIZE];
    let start = Instant::now();
    for _ in 0..iters {
        let mut va = Paragon5Voice::new_looped(
            0x0180,
            0x0002_8000,
            4096 << 16,
            0x0001_4000,
            3584 << 16,
            0x0100,
            0x00C0,
            0x0001_2000,
        );
        let mut vb = Paragon5Voice::new_looped(
            0x0120,
            0x0010_4000,
            4096 << 16,
            0x0000_C000,
            3584 << 16,
            0x0140,
            0x0080,
            0x0000_E000,
        );
        AccuracySynth::mix_voice_pair(
            black_box(&pcm_a),
            &mut va,
            black_box(&pcm_b),
            &mut vb,
            &mut track_buf,
        );
        black_box(&track_buf);
    }
    let elapsed = start.elapsed();
    println!(
        "AccuracySynth::mix_voice_pair (64f):                {:8.2} ns/iter ({:8.2} MSamples/sec)",
        elapsed.as_nanos() as f64 / iters as f64,
        ((iters * PARAGON5_BLOCK_SIZE) as f64 / elapsed.as_secs_f64()) / 1_000_000.0
    );

    // 2. AccuracySynth::sum_tracks (8 tracks x 64 frames)
    let tracks = [[32i8; PARAGON5_BLOCK_SIZE]; 8];
    let mut sum_out = [0i8; PARAGON5_BLOCK_SIZE];
    let start = Instant::now();
    for _ in 0..iters {
        let _ = AccuracySynth::sum_tracks(black_box(&tracks), &mut sum_out);
        black_box(&sum_out);
    }
    let elapsed = start.elapsed();
    println!(
        "AccuracySynth::sum_tracks (8 tracks, 64f):          {:8.2} ns/iter ({:8.2} MSamples/sec)",
        elapsed.as_nanos() as f64 / iters as f64,
        ((iters * PARAGON5_BLOCK_SIZE) as f64 / elapsed.as_secs_f64()) / 1_000_000.0
    );

    // 3. AccuracySynth::post_process_stereo (GbassV2 crossfeed, 128 frames)
    let mut left_128 = [64i8; 128];
    let mut right_128 = [-48i8; 128];
    let start = Instant::now();
    for _ in 0..iters {
        AccuracySynth::post_process_stereo(
            Paragon5Version::GbassV2,
            black_box(&mut left_128),
            black_box(&mut right_128),
        );
        black_box((&left_128, &right_128));
    }
    let elapsed = start.elapsed();
    println!(
        "AccuracySynth::gbass_v2_crossfeed (128f):           {:8.2} ns/iter ({:8.2} MSamples/sec)",
        elapsed.as_nanos() as f64 / iters as f64,
        ((iters * 128) as f64 / elapsed.as_secs_f64()) / 1_000_000.0
    );

    // 4. StudioSynth 48 kHz Lanczos-3 (1024 frames)
    let studio = StudioSynth::default();
    let mut out_l = vec![0.0f32; 1024];
    let mut out_r = vec![0.0f32; 1024];
    let studio_iters = 50_000usize;
    let start = Instant::now();
    for _ in 0..studio_iters {
        let mut pos = 0.0f64;
        studio.render_voice_f32(
            black_box(&sample),
            &mut pos,
            13_379.0,
            0.85,
            0.25,
            &mut out_l,
            &mut out_r,
        );
        black_box((&out_l, &out_r));
    }
    let elapsed = start.elapsed();
    println!(
        "StudioSynth::render_voice_f32 (48kHz Lanczos3, 1k): {:8.2} ns/iter ({:8.2} MSamples/sec)",
        elapsed.as_nanos() as f64 / studio_iters as f64,
        ((studio_iters * 1024) as f64 / elapsed.as_secs_f64()) / 1_000_000.0
    );
}
