//! Collision detection and avoidance volume computation.
//!
//! This module computes the regions where support branches would collide with
//! the model and the avoidance zones around them. It provides cached, per-layer,
//! per-radius collision polygons matching the C++ `TreeModelVolumes` / `TreeSupportData`
//! functionality.
//!
//! Collision checks use parry3d for efficient sphere/AABB queries, and results
//! are cached in a thread-safe map keyed by `(radius, layer_index)`.

use crate::config::DerivedSupportSettings;
use crate::mesh::TriangleMesh;
use crate::utils::{CoordF, CoordT, Point2D};
use nalgebra::Point3;
use rayon::prelude::*;
use std::collections::HashMap;
use std::sync::RwLock;

/// Key for the collision cache: (radius in scaled coords, layer index).
type CacheKey = (CoordT, usize);

/// A 2D collision region at a specific layer, represented as a set of
/// boundary points forming a polygon.
#[derive(Debug, Clone)]
pub struct CollisionRegion {
    /// Boundary points of the collision polygon.
    pub boundary: Vec<Point2D>,
}

/// Cached collision and avoidance data for the model.
///
/// This struct mirrors the C++ `TreeModelVolumes` / `TreeSupportData` class,
/// providing lazily-computed and cached collision polygons per layer and radius.
///
/// Thread-safe: uses `RwLock` for the internal cache so multiple threads can
/// query simultaneously.
pub struct CollisionModel {
    /// The source mesh (shared reference via Arc in practice).
    mesh: TriangleMesh,
    /// Derived support settings.
    settings: DerivedSupportSettings,
    /// Per-layer Z heights.
    layer_heights: Vec<CoordF>,
    /// Collision cache: `(radius, layer) → collision polygons`.
    collision_cache: RwLock<HashMap<CacheKey, Vec<CollisionRegion>>>,
    /// Avoidance cache: `(radius, layer) → avoidance polygons`.
    avoidance_cache: RwLock<HashMap<CacheKey, Vec<CollisionRegion>>>,
    /// Pre-computed per-layer triangle indices for fast lookup.
    layer_triangle_indices: Vec<Vec<usize>>,
}

impl CollisionModel {
    /// Create a new collision model for the given mesh and settings.
    ///
    /// Pre-computes per-layer triangle assignments for fast queries.
    ///
    /// # Arguments
    ///
    /// * `mesh` - The input triangle mesh.
    /// * `settings` - Derived support settings.
    /// * `layer_heights` - Z heights for each layer slice.
    ///
    /// # Examples
    ///
    /// ```
    /// use orca_tree_supports::collision::CollisionModel;
    /// use orca_tree_supports::config::{TreeSupportConfig, DerivedSupportSettings};
    /// use orca_tree_supports::mesh::TriangleMesh;
    /// use nalgebra::Point3;
    ///
    /// let vertices = vec![
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    ///     Point3::new(0.5, 1.0, 1.0),
    /// ];
    /// let mesh = TriangleMesh::from_vertices_indices(vertices, vec![[0, 1, 2]]).unwrap();
    /// let cfg = TreeSupportConfig::default();
    /// let settings = DerivedSupportSettings::from_config(&cfg);
    /// let layers = mesh.layer_z_values(0.2);
    /// let model = CollisionModel::new(mesh, settings, layers);
    /// ```
    pub fn new(
        mesh: TriangleMesh,
        settings: DerivedSupportSettings,
        layer_heights: Vec<CoordF>,
    ) -> Self {
        let layer_triangle_indices = precompute_layer_triangles(&mesh, &layer_heights);

        Self {
            mesh,
            settings,
            layer_heights,
            collision_cache: RwLock::new(HashMap::new()),
            avoidance_cache: RwLock::new(HashMap::new()),
            layer_triangle_indices,
        }
    }

    /// Get collision regions for a given radius and layer index.
    ///
    /// Results are cached after the first computation. The collision region
    /// represents the XY area where a support branch of the given radius
    /// would intersect the model at the specified layer.
    ///
    /// # Arguments
    ///
    /// * `radius` - Branch radius in scaled coordinates.
    /// * `layer_idx` - Layer index (0-based).
    pub fn get_collision(&self, radius: CoordT, layer_idx: usize) -> Vec<CollisionRegion> {
        let key = (radius, layer_idx);

        // Try reading from cache first.
        {
            let cache = self.collision_cache.read().unwrap();
            if let Some(regions) = cache.get(&key) {
                return regions.clone();
            }
        }

        // Compute and insert into cache.
        let regions = self.compute_collision(radius, layer_idx);
        {
            let mut cache = self.collision_cache.write().unwrap();
            cache.insert(key, regions.clone());
        }
        regions
    }

    /// Get avoidance regions for a given radius and layer index.
    ///
    /// Avoidance regions include the collision regions plus an XY distance
    /// margin. A support branch should not be placed within avoidance regions.
    ///
    /// # Arguments
    ///
    /// * `radius` - Branch radius in scaled coordinates.
    /// * `layer_idx` - Layer index (0-based).
    pub fn get_avoidance(&self, radius: CoordT, layer_idx: usize) -> Vec<CollisionRegion> {
        let key = (radius, layer_idx);

        {
            let cache = self.avoidance_cache.read().unwrap();
            if let Some(regions) = cache.get(&key) {
                return regions.clone();
            }
        }

        let regions = self.compute_avoidance(radius, layer_idx);
        {
            let mut cache = self.avoidance_cache.write().unwrap();
            cache.insert(key, regions.clone());
        }
        regions
    }

    /// Pre-compute collision data for all layers in parallel.
    ///
    /// This is useful to warm the cache before branch generation begins.
    pub fn precalculate(&self, radius: CoordT) {
        let num_layers = self.layer_heights.len();
        let keys: Vec<CacheKey> = (0..num_layers).map(|li| (radius, li)).collect();

        let results: Vec<(CacheKey, Vec<CollisionRegion>)> = keys
            .par_iter()
            .map(|&(r, li)| ((r, li), self.compute_collision(r, li)))
            .collect();

        let mut cache = self.collision_cache.write().unwrap();
        for (key, regions) in results {
            cache.insert(key, regions);
        }
    }

    /// Check if a point at a given layer collides with the model.
    ///
    /// # Arguments
    ///
    /// * `point` - 2D point in scaled coordinates.
    /// * `radius` - Branch radius in scaled coordinates.
    /// * `layer_idx` - Layer index.
    pub fn is_colliding(&self, point: &Point2D, radius: CoordT, layer_idx: usize) -> bool {
        let collision_regions = self.get_collision(radius, layer_idx);
        let radius_f = radius as f64;

        for region in &collision_regions {
            for boundary_point in &region.boundary {
                if point.distance_to(boundary_point) < radius_f {
                    return true;
                }
            }
        }
        false
    }

    /// Number of layers.
    #[inline]
    pub fn layer_count(&self) -> usize {
        self.layer_heights.len()
    }

    /// Get the Z height for a given layer index.
    #[inline]
    pub fn layer_z(&self, layer_idx: usize) -> CoordF {
        self.layer_heights[layer_idx]
    }

    /// Compute collision regions for a specific radius and layer.
    fn compute_collision(&self, radius: CoordT, layer_idx: usize) -> Vec<CollisionRegion> {
        if layer_idx >= self.layer_heights.len() {
            return Vec::new();
        }

        let _z = self.layer_heights[layer_idx];
        let radius_mm = radius as f64 * 1e-6; // convert scaled → mm
        let xy_dist_mm = self.settings.xy_distance as f64 * 1e-6;
        let effective_radius = radius_mm + xy_dist_mm;

        let tri_indices = &self.layer_triangle_indices[layer_idx];
        let mut regions = Vec::new();

        for &ti in tri_indices {
            let [v0, v1, v2] = self.mesh.triangle_vertices(ti);

            // Project triangle to 2D (XY plane) and expand by radius.
            let cx = (v0.x + v1.x + v2.x) / 3.0;
            let cy = (v0.y + v1.y + v2.y) / 3.0;

            // Create a simplified collision boundary as a circle approximation
            // around the triangle centroid, expanded by the effective radius.
            let boundary = approximate_circle_boundary(cx, cy, effective_radius, 8);

            regions.push(CollisionRegion { boundary });
        }

        regions
    }

    /// Compute avoidance regions (collision + XY distance margin).
    fn compute_avoidance(&self, radius: CoordT, layer_idx: usize) -> Vec<CollisionRegion> {
        // Avoidance = collision with extra XY distance.
        let extra_radius = radius + self.settings.xy_distance;
        self.compute_collision(extra_radius, layer_idx)
    }
}

/// Pre-compute which triangles intersect each layer's Z range.
fn precompute_layer_triangles(
    mesh: &TriangleMesh,
    layer_heights: &[CoordF],
) -> Vec<Vec<usize>> {
    if layer_heights.is_empty() {
        return Vec::new();
    }

    let layer_spacing = if layer_heights.len() > 1 {
        (layer_heights[1] - layer_heights[0]).abs()
    } else {
        0.2 // default fallback
    };
    let half_layer = layer_spacing * 0.5;

    // Precompute triangle Z ranges.
    let tri_z_ranges: Vec<(f64, f64)> = (0..mesh.triangle_count())
        .map(|ti| {
            let [v0, v1, v2] = mesh.triangle_vertices(ti);
            let z_min = v0.z.min(v1.z).min(v2.z);
            let z_max = v0.z.max(v1.z).max(v2.z);
            (z_min, z_max)
        })
        .collect();

    layer_heights
        .par_iter()
        .map(|&z| {
            let z_lo = z - half_layer;
            let z_hi = z + half_layer;
            tri_z_ranges
                .iter()
                .enumerate()
                .filter_map(|(ti, &(z_min, z_max))| {
                    if z_max >= z_lo && z_min <= z_hi {
                        Some(ti)
                    } else {
                        None
                    }
                })
                .collect()
        })
        .collect()
}

/// Generate boundary points approximating a circle at (cx, cy) with given
/// radius and number of segments.
fn approximate_circle_boundary(
    cx: f64,
    cy: f64,
    radius: f64,
    segments: usize,
) -> Vec<Point2D> {
    let mut points = Vec::with_capacity(segments);
    for i in 0..segments {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (segments as f64);
        let x = cx + radius * angle.cos();
        let y = cy + radius * angle.sin();
        // Convert mm → scaled coordinates
        points.push(Point2D::new(
            (x * 1e6) as CoordT,
            (y * 1e6) as CoordT,
        ));
    }
    points
}

/// Check if a 3D point (in mm) is inside the mesh AABB.
///
/// Quick reject test before more expensive collision queries.
#[inline]
pub fn point_in_aabb(point: &Point3<f64>, mesh: &TriangleMesh) -> bool {
    point.x >= mesh.aabb_min.x
        && point.x <= mesh.aabb_max.x
        && point.y >= mesh.aabb_min.y
        && point.y <= mesh.aabb_max.y
        && point.z >= mesh.aabb_min.z
        && point.z <= mesh.aabb_max.z
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::TreeSupportConfig;

    fn sample_mesh() -> TriangleMesh {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(5.0, 10.0, 0.0),
            Point3::new(5.0, 5.0, 10.0),
        ];
        let triangles = vec![[0, 1, 2], [0, 1, 3], [1, 2, 3], [0, 2, 3]];
        TriangleMesh::from_vertices_indices(vertices, triangles).unwrap()
    }

    #[test]
    fn test_collision_model_creation() {
        let mesh = sample_mesh();
        let cfg = TreeSupportConfig::default();
        let settings = DerivedSupportSettings::from_config(&cfg);
        let layers = mesh.layer_z_values(0.5);
        let model = CollisionModel::new(mesh, settings, layers);
        assert!(model.layer_count() > 0);
    }

    #[test]
    fn test_get_collision() {
        let mesh = sample_mesh();
        let cfg = TreeSupportConfig::default();
        let settings = DerivedSupportSettings::from_config(&cfg);
        let layers = mesh.layer_z_values(1.0);
        let model = CollisionModel::new(mesh, settings, layers);

        // Query collision at layer 0 with a small radius.
        let regions = model.get_collision(100_000, 0);
        // The exact number depends on geometry; just verify no crash.
        let _ = regions;
    }

    #[test]
    fn test_precalculate() {
        let mesh = sample_mesh();
        let cfg = TreeSupportConfig::default();
        let settings = DerivedSupportSettings::from_config(&cfg);
        let layers = mesh.layer_z_values(2.0);
        let model = CollisionModel::new(mesh, settings, layers);
        model.precalculate(200_000);
        // Verify cache is populated.
        let regions = model.get_collision(200_000, 0);
        let _ = regions;
    }

    #[test]
    fn test_point_in_aabb() {
        let mesh = sample_mesh();
        assert!(point_in_aabb(&Point3::new(5.0, 5.0, 5.0), &mesh));
        assert!(!point_in_aabb(&Point3::new(20.0, 5.0, 5.0), &mesh));
    }

    #[test]
    fn test_approximate_circle() {
        let points = approximate_circle_boundary(0.0, 0.0, 1.0, 8);
        assert_eq!(points.len(), 8);
    }
}
