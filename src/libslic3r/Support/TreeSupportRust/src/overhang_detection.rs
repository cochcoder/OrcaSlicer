//! Overhang detection for tree support generation.
//!
//! Identifies regions of the mesh that require support by analyzing triangle
//! face normals and detecting overhangs beyond the configured angle threshold.
//!
//! The detection is parallelized per-layer using Rayon for high performance on
//! complex models.

use crate::config::TreeSupportConfig;
use crate::mesh::TriangleMesh;
use crate::utils::{CoordF, Point2D};
use nalgebra::Vector3;
use rayon::prelude::*;

/// A contiguous region requiring support at a given layer.
///
/// Stored as a collection of 2D boundary points (the overhang polygon outline)
/// at a specific Z height.
#[derive(Debug, Clone)]
pub struct OverhangArea {
    /// Z height of this overhang layer.
    pub z: CoordF,
    /// Layer index (0-based from bottom).
    pub layer_index: usize,
    /// Boundary points of the overhang region (scaled 2D coordinates).
    pub boundary: Vec<Point2D>,
    /// Area of the overhang region in scaled² units.
    pub area: f64,
}

/// Result of overhang detection: per-layer overhang areas.
#[derive(Debug, Clone)]
pub struct OverhangResult {
    /// Overhang areas grouped by layer, sorted by layer index.
    pub layers: Vec<Vec<OverhangArea>>,
    /// Total number of overhang regions detected.
    pub total_regions: usize,
}

/// Detect overhangs on the mesh that need tree support.
///
/// Identifies triangles whose face normal exceeds the support angle threshold,
/// meaning they overhang too much to print without support. Results are grouped
/// by layer.
///
/// This function is parallelized across layers using Rayon.
///
/// # Arguments
///
/// * `mesh` - The input triangle mesh.
/// * `config` - Support configuration (angle threshold, layer height, etc.).
/// * `layer_heights` - Z heights for each layer slice.
///
/// # Examples
///
/// ```
/// use orca_tree_supports::overhang_detection::detect_overhangs;
/// use orca_tree_supports::config::TreeSupportConfig;
/// use orca_tree_supports::mesh::TriangleMesh;
/// use nalgebra::Point3;
///
/// let vertices = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(0.5, 0.0, 1.0),
/// ];
/// let mesh = TriangleMesh::from_vertices_indices(vertices, vec![[0, 1, 2]]).unwrap();
/// let cfg = TreeSupportConfig::default();
/// let layer_heights = mesh.layer_z_values(0.2);
/// let result = detect_overhangs(&mesh, &cfg, &layer_heights);
/// assert!(result.layers.len() == layer_heights.len());
/// ```
pub fn detect_overhangs(
    mesh: &TriangleMesh,
    config: &TreeSupportConfig,
    layer_heights: &[CoordF],
) -> OverhangResult {
    let cos_threshold = config.support_angle.cos();
    let up = Vector3::new(0.0, 0.0, 1.0);

    // Pre-classify triangles as overhang or not.
    let overhang_triangles: Vec<(usize, CoordF, CoordF)> = (0..mesh.triangle_count())
        .filter_map(|ti| {
            let normal = mesh.triangle_normal(ti);
            let norm_len = normal.norm();
            if norm_len < 1e-12 {
                return None; // degenerate triangle
            }
            let cos_angle = normal.dot(&up) / norm_len;
            // An overhang occurs when the face normal points downward more than
            // the threshold. cos_angle < cos_threshold for angles measured from
            // the vertical (Z-up).
            if cos_angle < cos_threshold {
                let [v0, v1, v2] = mesh.triangle_vertices(ti);
                let z_min = v0.z.min(v1.z).min(v2.z);
                let z_max = v0.z.max(v1.z).max(v2.z);
                Some((ti, z_min, z_max))
            } else {
                None
            }
        })
        .collect();

    // For each layer, find which overhang triangles intersect that Z level.
    let layers: Vec<Vec<OverhangArea>> = layer_heights
        .par_iter()
        .enumerate()
        .map(|(layer_idx, &z)| {
            let half_layer = if !layer_heights.is_empty() && layer_heights.len() > 1 {
                (layer_heights[1] - layer_heights[0]).abs() * 0.5
            } else {
                0.1
            };
            let z_lo = z - half_layer;
            let z_hi = z + half_layer;

            let mut areas = Vec::new();

            for &(ti, tri_z_min, tri_z_max) in &overhang_triangles {
                // Check if triangle spans this layer.
                if tri_z_max < z_lo || tri_z_min > z_hi {
                    continue;
                }

                let [v0, v1, v2] = mesh.triangle_vertices(ti);
                // Project triangle centroid to 2D as a simple point representation.
                let cx = ((v0.x + v1.x + v2.x) / 3.0 * 1e6) as i64;
                let cy = ((v0.y + v1.y + v2.y) / 3.0 * 1e6) as i64;

                // Compute approximate projected area.
                let edge1_x = v1.x - v0.x;
                let edge1_y = v1.y - v0.y;
                let edge2_x = v2.x - v0.x;
                let edge2_y = v2.y - v0.y;
                let area_2d = (edge1_x * edge2_y - edge1_y * edge2_x).abs() * 0.5;

                areas.push(OverhangArea {
                    z,
                    layer_index: layer_idx,
                    boundary: vec![Point2D::new(cx, cy)],
                    area: area_2d * 1e12, // convert to scaled² units
                });
            }

            areas
        })
        .collect();

    let total_regions: usize = layers.iter().map(|l| l.len()).sum();

    OverhangResult {
        layers,
        total_regions,
    }
}

/// Check if a single triangle is an overhang given the support angle.
///
/// Returns `true` if the triangle's face normal deviates from vertical more
/// than `support_angle` radians.
///
/// # Arguments
///
/// * `normal` - The (unnormalized) face normal of the triangle.
/// * `support_angle` - Maximum overhang angle in radians.
#[inline]
pub fn is_overhang(normal: &Vector3<f64>, support_angle: f64) -> bool {
    let norm_len = normal.norm();
    if norm_len < 1e-12 {
        return false;
    }
    let cos_angle = normal.z / norm_len;
    cos_angle < support_angle.cos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;
    use std::f64::consts::PI;

    #[test]
    fn test_is_overhang_flat_top() {
        // Flat top face: normal = (0, 0, 1), never an overhang
        let normal = Vector3::new(0.0, 0.0, 1.0);
        assert!(!is_overhang(&normal, 50.0 * PI / 180.0));
    }

    #[test]
    fn test_is_overhang_flat_bottom() {
        // Flat bottom face: normal = (0, 0, -1), always an overhang
        let normal = Vector3::new(0.0, 0.0, -1.0);
        assert!(is_overhang(&normal, 50.0 * PI / 180.0));
    }

    #[test]
    fn test_is_overhang_vertical() {
        // Vertical face: normal = (1, 0, 0), angle = 90°
        let normal = Vector3::new(1.0, 0.0, 0.0);
        // With a 50° threshold, a 90° face is an overhang
        assert!(is_overhang(&normal, 50.0 * PI / 180.0));
    }

    #[test]
    fn test_is_overhang_45_degrees() {
        // 45° face: normal = (1, 0, 1)
        let normal = Vector3::new(1.0, 0.0, 1.0);
        // With 50° threshold (cos(50°) ≈ 0.643), cos(45°) ≈ 0.707 > 0.643 → not overhang
        assert!(!is_overhang(&normal, 50.0 * PI / 180.0));
        // With 40° threshold (cos(40°) ≈ 0.766), cos(45°) ≈ 0.707 < 0.766 → overhang
        assert!(is_overhang(&normal, 40.0 * PI / 180.0));
    }

    #[test]
    fn test_detect_overhangs_simple() {
        // A horizontal overhang triangle at z=0.5
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.5),
            Point3::new(1.0, 0.0, 0.5),
            Point3::new(0.5, 1.0, 0.5),
        ];
        let mesh = TriangleMesh::from_vertices_indices(vertices, vec![[0, 1, 2]]).unwrap();
        let cfg = TreeSupportConfig::default();
        let layer_heights = vec![0.5];
        let result = detect_overhangs(&mesh, &cfg, &layer_heights);
        assert_eq!(result.layers.len(), 1);
        // A flat horizontal face (normal pointing up) is NOT an overhang
        // since cos(0°) = 1.0 > cos(50°) ≈ 0.643
    }

    #[test]
    fn test_detect_overhangs_empty_mesh() {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 1.0),
        ];
        let mesh = TriangleMesh::from_vertices_indices(vertices, vec![[0, 1, 2]]).unwrap();
        let cfg = TreeSupportConfig::default();
        let result = detect_overhangs(&mesh, &cfg, &[]);
        assert_eq!(result.layers.len(), 0);
        assert_eq!(result.total_regions, 0);
    }
}
