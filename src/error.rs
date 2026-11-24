//! Error types for Paragon 5 / GBASS parsing and synthesis.

use core::fmt;

/// Errors returned when parsing Paragon 5 / GBASS headers, sample descriptors, or mixing audio.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Error {
    /// Input buffer ended before a complete header or payload could be read.
    UnexpectedEof,
    /// Master header or sample descriptor failed structural validation.
    InvalidHeader(&'static str),
    /// Loop range exceeded or violated the sample length bounds.
    InvalidLoopRange {
        /// Loop start offset in samples.
        loop_start: u32,
        /// Loop end offset in samples.
        loop_end: u32,
        /// Total PCM sample length in bytes.
        pcm_length: usize,
    },
    /// Sample index was out of range for the sample descriptor table.
    InvalidSample(u32),
    /// Track count passed to the multi-track summing mixer was outside `1..=8`.
    InvalidTrackCount(usize),
    /// Invalid synthesis parameter.
    InvalidParameter(&'static str),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnexpectedEof => {
                write!(f, "Unexpected end of data while parsing Paragon 5 stream")
            }
            Self::InvalidHeader(msg) => write!(f, "Invalid Paragon 5 header: {msg}"),
            Self::InvalidLoopRange {
                loop_start,
                loop_end,
                pcm_length,
            } => write!(
                f,
                "Invalid Paragon 5 loop range {loop_start}..{loop_end} for {pcm_length}-byte sample"
            ),
            Self::InvalidSample(id) => write!(f, "Invalid Paragon 5 sample index {id}"),
            Self::InvalidTrackCount(n) => {
                write!(f, "Invalid Paragon 5 summing track count {n} (expected 1..=8)")
            }
            Self::InvalidParameter(msg) => write!(f, "Invalid Paragon 5 parameter: {msg}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

/// Convenience result alias for `paragon5`.
pub type Result<T> = core::result::Result<T, Error>;
