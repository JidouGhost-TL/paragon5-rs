//! Paragon 5 / GBASS master audio header, 24-byte 16.16 fixed-point sample descriptor,
//! and zero-allocation signed 8-bit PCM sample views.

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

use crate::error::{Error, Result};
use crate::freq::{PARAGON5_DEFAULT_MIX_RATE_HZ, Q16_ONE};

/// GBA ROM bus window base address (`0x0800_0000`).
pub const GBA_ROM_BUS_BASE: u32 = 0x0800_0000;

/// Size of a Paragon 5 / GBASS `Paragon5SampleDesc` descriptor in bytes (`0x18` = 24 bytes).
pub const PARAGON5_SAMPLE_DESC_SIZE: usize = 24;

/// Size of a Paragon 5 / GBASS `Paragon5MasterHeader` in bytes (`0x40` = 64 bytes).
pub const PARAGON5_MASTER_HEADER_SIZE: usize = 64;

/// Standard unity 16.16 pitch scale multiplier (`0x0001_0000`) stored at `+0x04` of every
/// `Paragon5SampleDesc`.
pub const PARAGON5_PITCH_SCALE_ONE: u32 = Q16_ONE;

/// Parsed 64-byte (`0x40`) Paragon 5 / GBASS master audio header.
///
/// The master header locates the song table (`+0x08`), sample count (`+0x18`), and contiguous
/// 24-byte sample descriptor array (`+0x1C`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Paragon5MasterHeader {
    /// Header word `+0x00` (engine version/flags in GBASS v2; pointer in GBASS v1).
    pub flags_or_ptr: u32,
    /// Header word `+0x04` (song count in GBASS v2; `0` in GBASS v1).
    pub num_songs: u32,
    /// Header word `+0x08`: GBA ROM pointer (`0x08xxxxxx`) to the song table.
    pub song_table_ptr: u32,
    /// Header word `+0x18`: number of 24-byte [`Paragon5SampleDesc`] entries.
    pub num_samples: u32,
    /// Header word `+0x1C`: GBA ROM pointer (`0x08xxxxxx`) to `[Paragon5SampleDesc; num_samples]`.
    pub sample_table_ptr: u32,
}

impl Paragon5MasterHeader {
    /// Parse a 64-byte Paragon 5 / GBASS master header from raw little-endian bytes.
    pub fn parse(data: &[u8]) -> Result<Self> {
        if data.len() < PARAGON5_MASTER_HEADER_SIZE {
            return Err(Error::UnexpectedEof);
        }
        let flags_or_ptr = read_u32_le(data, 0x00);
        let num_songs = read_u32_le(data, 0x04);
        let song_table_ptr = read_u32_le(data, 0x08);
        let num_samples = read_u32_le(data, 0x18);
        let sample_table_ptr = read_u32_le(data, 0x1C);

        if num_samples == 0 || num_samples > 4096 {
            return Err(Error::InvalidHeader("num_samples out of valid range"));
        }
        if sample_table_ptr < GBA_ROM_BUS_BASE {
            return Err(Error::InvalidHeader("sample_table_ptr below GBA ROM bus"));
        }

        Ok(Self {
            flags_or_ptr,
            num_songs,
            song_table_ptr,
            num_samples,
            sample_table_ptr,
        })
    }

    /// Convert `sample_table_ptr` to a zero-based ROM file offset.
    #[inline]
    #[must_use]
    pub const fn sample_table_offset(&self) -> usize {
        (self.sample_table_ptr - GBA_ROM_BUS_BASE) as usize
    }

    /// Convert `song_table_ptr` to a zero-based ROM file offset, if it lies in GBA ROM space.
    #[inline]
    #[must_use]
    pub const fn song_table_offset(&self) -> Option<usize> {
        if self.song_table_ptr >= GBA_ROM_BUS_BASE {
            Some((self.song_table_ptr - GBA_ROM_BUS_BASE) as usize)
        } else {
            None
        }
    }
}

/// Parsed 24-byte (`0x18`) Paragon 5 / GBASS 16.16 fixed-point PCM8 sample descriptor.
///
/// Binary layout (little-endian):
/// - `+0x00`: `sample_ptr: u32` — GBA ROM pointer (`0x08xxxxxx`) to raw signed 8-bit PCM (`i8`).
/// - `+0x04`: `pitch_scale_fp16: u32` — 16.16 fixed-point pitch multiplier (`0x0001_0000` = 1.0x).
/// - `+0x08`: `reserved_zero: u32` — Always `0`.
/// - `+0x0C`: `length_fp16: u32` — Sample length in 16.16 fixed-point (`bytes = length_fp16 >> 16`).
/// - `+0x10`: `loop_start_fp16: u32` — Loop start in 16.16 fixed-point (`bytes = loop_start_fp16 >> 16`).
/// - `+0x14`: `loop_end_fp16: u32` — Loop end in 16.16 fixed-point (`bytes = loop_end_fp16 >> 16`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Paragon5SampleDesc {
    /// GBA ROM pointer (`0x08xxxxxx`) to raw signed 8-bit linear PCM (`i8`).
    pub sample_ptr: u32,
    /// 16.16 fixed-point rate multiplier (typically `0x0001_0000` = `1.0x`).
    pub pitch_scale_fp16: u32,
    /// Reserved zero field (`0`).
    pub reserved_zero: u32,
    /// Sample length in 16.16 fixed-point.
    pub length_fp16: u32,
    /// Loop start offset in 16.16 fixed-point (`0` for unlooped samples).
    pub loop_start_fp16: u32,
    /// Loop end offset in 16.16 fixed-point.
    pub loop_end_fp16: u32,
}

impl Paragon5SampleDesc {
    /// Parse a 24-byte `Paragon5SampleDesc` from raw little-endian bytes.
    pub fn parse(data: &[u8]) -> Result<Self> {
        if data.len() < PARAGON5_SAMPLE_DESC_SIZE {
            return Err(Error::UnexpectedEof);
        }
        let sample_ptr = read_u32_le(data, 0x00);
        let pitch_scale_fp16 = read_u32_le(data, 0x04);
        let reserved_zero = read_u32_le(data, 0x08);
        let length_fp16 = read_u32_le(data, 0x0C);
        let loop_start_fp16 = read_u32_le(data, 0x10);
        let loop_end_fp16 = read_u32_le(data, 0x14);

        if sample_ptr < GBA_ROM_BUS_BASE {
            return Err(Error::InvalidHeader("sample_ptr below GBA ROM bus"));
        }
        if pitch_scale_fp16 == 0 {
            return Err(Error::InvalidHeader("pitch_scale_fp16 is zero"));
        }
        if reserved_zero != 0 {
            return Err(Error::InvalidHeader("reserved_zero field is non-zero"));
        }
        let len = length_fp16 >> 16;
        let ls = loop_start_fp16 >> 16;
        let le = loop_end_fp16 >> 16;
        if len == 0 {
            return Err(Error::InvalidHeader("sample length is zero"));
        }
        if ls > le {
            return Err(Error::InvalidLoopRange {
                loop_start: ls,
                loop_end: le,
                pcm_length: len as usize,
            });
        }

        Ok(Self {
            sample_ptr,
            pitch_scale_fp16,
            reserved_zero,
            length_fp16,
            loop_start_fp16,
            loop_end_fp16,
        })
    }

    /// Zero-based ROM file offset of the signed 8-bit PCM payload.
    #[inline]
    #[must_use]
    pub const fn sample_rom_offset(&self) -> usize {
        (self.sample_ptr - GBA_ROM_BUS_BASE) as usize
    }

    /// Integer sample length in bytes (`length_fp16 >> 16`).
    #[inline]
    #[must_use]
    pub const fn length(&self) -> usize {
        (self.length_fp16 >> 16) as usize
    }

    /// Integer loop start offset in bytes (`loop_start_fp16 >> 16`).
    #[inline]
    #[must_use]
    pub const fn loop_start(&self) -> u32 {
        self.loop_start_fp16 >> 16
    }

    /// Integer loop end offset in bytes, clamped to [`Self::length`].
    #[inline]
    #[must_use]
    pub const fn loop_end(&self) -> u32 {
        let len = self.length_fp16 >> 16;
        let le = self.loop_end_fp16 >> 16;
        if le < len { le } else { len }
    }

    /// Returns `true` if this sample has a valid sustain loop (`loop_start > 0 && loop_end > loop_start`).
    #[inline]
    #[must_use]
    pub const fn is_looped(&self) -> bool {
        let ls = self.loop_start();
        let le = self.loop_end();
        ls > 0 && le > ls
    }

    /// Loop length in 16.16 fixed-point (`loop_end_fp16 - loop_start_fp16`), or `0` if unlooped.
    #[inline]
    #[must_use]
    pub const fn loop_len_fp16(&self) -> u32 {
        if self.is_looped() {
            self.loop_end_fp16.saturating_sub(self.loop_start_fp16)
        } else {
            0
        }
    }

    /// Resolve this descriptor into a zero-allocation [`Paragon5SampleView`] borrowing directly
    /// from `rom`.
    pub fn resolve_view<'a>(&self, rom: &'a [u8]) -> Result<Paragon5SampleView<'a>> {
        let start = self.sample_rom_offset();
        let len = self.length();
        let end = start.checked_add(len).ok_or(Error::UnexpectedEof)?;
        if end > rom.len() {
            return Err(Error::UnexpectedEof);
        }
        let (loop_start, loop_end) = if self.is_looped() {
            (Some(self.loop_start()), Some(self.loop_end()))
        } else {
            (None, None)
        };
        Ok(Paragon5SampleView {
            pcm_bytes: &rom[start..end],
            sample_rate_hz: PARAGON5_DEFAULT_MIX_RATE_HZ,
            root_key: 60,
            pitch_scale_fp16: self.pitch_scale_fp16,
            loop_start,
            loop_end,
        })
    }
}

/// Zero-allocation borrowed view of a Paragon 5 / GBASS signed 8-bit PCM sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Paragon5SampleView<'a> {
    /// Raw signed 8-bit linear PCM bytes (`i8` reinterpreted as `u8`).
    pub pcm_bytes: &'a [u8],
    /// Native playback sample rate in Hz (default `13_379`).
    pub sample_rate_hz: u32,
    /// MIDI root key (`0..=127`; default `60` = C4).
    pub root_key: u8,
    /// 16.16 fixed-point pitch scale multiplier (`0x0001_0000` = `1.0x`).
    pub pitch_scale_fp16: u32,
    /// Optional loop start offset in samples.
    pub loop_start: Option<u32>,
    /// Optional loop end offset in samples.
    pub loop_end: Option<u32>,
}

impl<'a> Paragon5SampleView<'a> {
    /// Construct a `Paragon5SampleView` directly from raw signed 8-bit PCM bytes.
    #[inline]
    #[must_use]
    pub const fn from_pcm_bytes(
        sample_rate_hz: u32,
        root_key: u8,
        loop_start: Option<u32>,
        loop_end: Option<u32>,
        pcm_bytes: &'a [u8],
    ) -> Self {
        Self {
            pcm_bytes,
            sample_rate_hz,
            root_key,
            pitch_scale_fp16: PARAGON5_PITCH_SCALE_ONE,
            loop_start,
            loop_end,
        }
    }

    /// Number of signed 8-bit PCM samples.
    #[inline]
    #[must_use]
    pub const fn len(&self) -> usize {
        self.pcm_bytes.len()
    }

    /// Returns `true` if the sample contains no PCM bytes.
    #[inline]
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.pcm_bytes.is_empty()
    }

    /// Effective loop length in samples, or `0` if the sample is one-shot.
    #[inline]
    #[must_use]
    pub const fn loop_len(&self) -> usize {
        match (self.loop_start, self.loop_end) {
            (Some(ls), Some(le)) if le > ls && (le as usize) <= self.pcm_bytes.len() => {
                (le - ls) as usize
            }
            (Some(ls), None) if (ls as usize) < self.pcm_bytes.len() => {
                self.pcm_bytes.len() - (ls as usize)
            }
            _ => 0,
        }
    }

    /// Effective playback end offset in samples (`loop_end` if looped, else `len()`).
    #[inline]
    #[must_use]
    pub const fn effective_end(&self) -> usize {
        match self.loop_end {
            Some(le) if (le as usize) <= self.pcm_bytes.len() && le > 0 => le as usize,
            _ => self.pcm_bytes.len(),
        }
    }

    /// Clone into an owned [`Paragon5Sample`] (requires `alloc` feature).
    #[cfg(feature = "alloc")]
    #[must_use]
    pub fn to_owned(&self) -> Paragon5Sample {
        Paragon5Sample {
            pcm_bytes: self.pcm_bytes.to_vec(),
            sample_rate_hz: self.sample_rate_hz,
            root_key: self.root_key,
            pitch_scale_fp16: self.pitch_scale_fp16,
            loop_start: self.loop_start,
            loop_end: self.loop_end,
        }
    }
}

/// Owned Paragon 5 / GBASS signed 8-bit PCM sample (requires `alloc` feature).
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Paragon5Sample {
    /// Raw signed 8-bit linear PCM bytes (`i8` reinterpreted as `u8`).
    pub pcm_bytes: Vec<u8>,
    /// Native playback sample rate in Hz.
    pub sample_rate_hz: u32,
    /// MIDI root key (`0..=127`).
    pub root_key: u8,
    /// 16.16 fixed-point pitch scale multiplier.
    pub pitch_scale_fp16: u32,
    /// Optional loop start offset in samples.
    pub loop_start: Option<u32>,
    /// Optional loop end offset in samples.
    pub loop_end: Option<u32>,
}

#[cfg(feature = "alloc")]
impl Paragon5Sample {
    /// Borrow this owned sample as a [`Paragon5SampleView`].
    #[inline]
    #[must_use]
    pub fn as_view(&self) -> Paragon5SampleView<'_> {
        Paragon5SampleView {
            pcm_bytes: &self.pcm_bytes,
            sample_rate_hz: self.sample_rate_hz,
            root_key: self.root_key,
            pitch_scale_fp16: self.pitch_scale_fp16,
            loop_start: self.loop_start,
            loop_end: self.loop_end,
        }
    }
}

#[inline]
const fn read_u32_le(data: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([data[off], data[off + 1], data[off + 2], data[off + 3]])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_sample_desc_and_master_header() {
        let mut rom = [0u8; 256];
        // Write 24-byte descriptor at 0x40 pointing to PCM at 0x80 (len=64, loop 16..64)
        let words: [u32; 6] = [
            GBA_ROM_BUS_BASE + 0x80,
            PARAGON5_PITCH_SCALE_ONE,
            0,
            64 << 16,
            16 << 16,
            64 << 16,
        ];
        for (i, w) in words.iter().enumerate() {
            rom[0x40 + i * 4..0x44 + i * 4].copy_from_slice(&w.to_le_bytes());
        }
        // Write 64-byte master header at 0x00
        rom[0x00..0x04].copy_from_slice(&2u32.to_le_bytes());
        rom[0x04..0x08].copy_from_slice(&4u32.to_le_bytes());
        rom[0x08..0x0C].copy_from_slice(&(GBA_ROM_BUS_BASE + 0x60).to_le_bytes());
        rom[0x18..0x1C].copy_from_slice(&1u32.to_le_bytes());
        rom[0x1C..0x20].copy_from_slice(&(GBA_ROM_BUS_BASE + 0x40).to_le_bytes());

        let hdr = Paragon5MasterHeader::parse(&rom[0x00..0x40]).expect("master header");
        assert_eq!(hdr.num_samples, 1);
        assert_eq!(hdr.sample_table_offset(), 0x40);
        assert_eq!(hdr.song_table_offset(), Some(0x60));

        let desc = Paragon5SampleDesc::parse(&rom[0x40..0x58]).expect("sample desc");
        assert_eq!(desc.sample_rom_offset(), 0x80);
        assert_eq!(desc.length(), 64);
        assert!(desc.is_looped());
        assert_eq!(desc.loop_start(), 16);
        assert_eq!(desc.loop_end(), 64);
        assert_eq!(desc.loop_len_fp16(), 48 << 16);

        let view = desc.resolve_view(&rom).expect("view");
        assert_eq!(view.len(), 64);
        assert_eq!(view.loop_len(), 48);
    }
}
