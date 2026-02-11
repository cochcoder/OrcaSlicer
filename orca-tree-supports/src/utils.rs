//! Error types and utility helpers for the tree support crate.
//!
//! Provides a unified error type ([`TreeSupportError`]) used across all modules,
//! plus small helpers for profiling and coordinate conversion.

use thiserror::Error;

/// Unified error type for all tree support operations.
#[derive(Debug, Error)]
pub enum TreeSupportError {
    /// The input mesh is degenerate or empty.
    #[error("invalid mesh: {0}")]
    InvalidMesh(String),

    /// A configuration parameter is out of range.
    #[error("invalid config: {0}")]
    InvalidConfig(String),

    /// An internal algorithm invariant was violated.
    #[error("internal error: {0}")]
    Internal(String),

    /// The FFI caller passed a null pointer where one was not expected.
    #[error("null pointer passed to FFI function: {0}")]
    NullPointer(String),
}

/// Result alias used throughout the crate.
pub type Result<T> = std::result::Result<T, TreeSupportError>;

/// Coordinate type matching the C++ `coord_t` (scaled integer coordinates).
///
/// In OrcaSlicer, 1 unit = 1 nanometre (1 mm = 1_000_000 units) when using
/// the default `SCALING_FACTOR` of `1e-6`.
pub type CoordT = i64;

/// Floating-point coordinate type matching C++ `coordf_t`.
pub type CoordF = f64;

/// The scaling factor used to convert between millimetres and internal integer
/// coordinates. Matches the C++ `SCALING_FACTOR` constant.
pub const SCALING_FACTOR: f64 = 1e-6;

/// Convert millimetres to internal scaled coordinates.
///
/// # Examples
///
/// ```
/// use orca_tree_supports::utils::scaled;
/// assert_eq!(scaled(1.0), 1_000_000);
/// ```
#[inline]
pub fn scaled(mm: f64) -> CoordT {
    (mm / SCALING_FACTOR) as CoordT
}

/// Convert internal scaled coordinates back to millimetres.
///
/// # Examples
///
/// ```
/// use orca_tree_supports::utils::unscaled;
/// assert!((unscaled(1_000_000) - 1.0).abs() < 1e-9);
/// ```
#[inline]
pub fn unscaled(coord: CoordT) -> f64 {
    coord as f64 * SCALING_FACTOR
}

/// A simple scoped timer that logs elapsed time when dropped.
///
/// Used internally for profiling critical sections.
/// Enable with `RUST_LOG=debug` or similar.
#[cfg(debug_assertions)]
pub struct ScopedTimer {
    label: &'static str,
    start: std::time::Instant,
}

#[cfg(debug_assertions)]
impl ScopedTimer {
    /// Create and start a new timer with the given label.
    pub fn new(label: &'static str) -> Self {
        Self {
            label,
            start: std::time::Instant::now(),
        }
    }
}

#[cfg(debug_assertions)]
impl Drop for ScopedTimer {
    fn drop(&mut self) {
        eprintln!(
            "[tree-support] {} took {:.3}ms",
            self.label,
            self.start.elapsed().as_secs_f64() * 1000.0
        );
    }
}

/// No-op timer for release builds.
#[cfg(not(debug_assertions))]
pub struct ScopedTimer;

#[cfg(not(debug_assertions))]
impl ScopedTimer {
    /// Create a no-op timer (does nothing in release builds).
    #[inline(always)]
    pub fn new(_label: &'static str) -> Self {
        Self
    }
}

/// A 2D point in scaled integer coordinates, matching C++ `Point`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(C)]
pub struct Point2D {
    /// X coordinate in scaled units.
    pub x: CoordT,
    /// Y coordinate in scaled units.
    pub y: CoordT,
}

impl Point2D {
    /// Create a new 2D point.
    #[inline]
    pub fn new(x: CoordT, y: CoordT) -> Self {
        Self { x, y }
    }

    /// Euclidean distance to another point.
    #[inline]
    pub fn distance_to(&self, other: &Self) -> f64 {
        let dx = (self.x - other.x) as f64;
        let dy = (self.y - other.y) as f64;
        (dx * dx + dy * dy).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scaled_unscaled_roundtrip() {
        let mm = 2.5;
        let s = scaled(mm);
        let back = unscaled(s);
        assert!((back - mm).abs() < 1e-6);
    }

    #[test]
    fn test_scaled_values() {
        assert_eq!(scaled(0.0), 0);
        assert_eq!(scaled(1.0), 1_000_000);
        assert_eq!(scaled(0.4), 400_000);
    }

    #[test]
    fn test_point2d_distance() {
        let a = Point2D::new(0, 0);
        let b = Point2D::new(3_000_000, 4_000_000);
        let d = a.distance_to(&b);
        assert!((d - 5_000_000.0).abs() < 1.0);
    }
}
