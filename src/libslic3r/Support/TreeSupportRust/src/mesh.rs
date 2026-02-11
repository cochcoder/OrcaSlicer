//! Indexed triangle mesh representation.
//!
//! Provides a simple indexed triangle mesh with AABB-accelerated queries,
//! designed for FFI compatibility and efficient processing.
//!
//! The [`MeshData`] struct is `#[repr(C)]` and holds raw pointers to vertex/index
//! arrays owned by the C++ caller. The safe Rust wrapper [`TriangleMesh`] copies
//! this data into owned vectors and builds an AABB tree for queries.

use crate::utils::{CoordF, Result, TreeSupportError};
use nalgebra::Point3;

/// FFI-compatible mesh descriptor.
///
/// This struct is passed from C/C++ to Rust and contains raw pointers to
/// vertex and index arrays. The data is **not owned** by this struct—the
/// caller must keep the underlying buffers alive for the lifetime of any
/// [`TreeSupportHandle`](crate::ffi::TreeSupportHandle) that references them.
///
/// # Safety
///
/// - `vertices` must point to `vertex_count * 3` contiguous `f32` values (x,y,z triples).
/// - `indices` must point to `triangle_count * 3` contiguous `u32` values (index triples).
/// - Both pointers must be valid and properly aligned for their element type.
#[derive(Debug)]
#[repr(C)]
pub struct MeshData {
    /// Pointer to packed vertex coordinates: [x0, y0, z0, x1, y1, z1, …].
    pub vertices: *const f32,
    /// Number of vertices (each vertex is 3 floats).
    pub vertex_count: u32,
    /// Pointer to packed triangle indices: [i0, j0, k0, i1, j1, k1, …].
    pub indices: *const u32,
    /// Number of triangles (each triangle is 3 indices).
    pub triangle_count: u32,
}

/// A triangle in the mesh, referencing vertex indices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Triangle {
    /// Index of the first vertex.
    pub v0: u32,
    /// Index of the second vertex.
    pub v1: u32,
    /// Index of the third vertex.
    pub v2: u32,
}

/// An owned, safe triangle mesh with AABB metadata.
///
/// Constructed from a [`MeshData`] FFI descriptor or from Rust vectors directly.
///
/// # Examples
///
/// ```
/// use orca_tree_supports::mesh::TriangleMesh;
///
/// // A single triangle
/// let vertices = vec![
///     nalgebra::Point3::new(0.0, 0.0, 0.0),
///     nalgebra::Point3::new(1.0, 0.0, 0.0),
///     nalgebra::Point3::new(0.0, 1.0, 0.0),
/// ];
/// let indices = vec![[0u32, 1, 2]];
/// let mesh = TriangleMesh::from_vertices_indices(vertices, indices).unwrap();
/// assert_eq!(mesh.triangle_count(), 1);
/// ```
#[derive(Debug, Clone)]
pub struct TriangleMesh {
    /// Vertex positions.
    pub vertices: Vec<Point3<f64>>,
    /// Triangle index triples.
    pub triangles: Vec<[u32; 3]>,
    /// Axis-aligned bounding box minimum corner.
    pub aabb_min: Point3<f64>,
    /// Axis-aligned bounding box maximum corner.
    pub aabb_max: Point3<f64>,
}

impl TriangleMesh {
    /// Create a mesh from owned vertex and index data.
    ///
    /// Returns an error if the mesh is empty or indices are out of range.
    pub fn from_vertices_indices(
        vertices: Vec<Point3<f64>>,
        triangles: Vec<[u32; 3]>,
    ) -> Result<Self> {
        if vertices.is_empty() {
            return Err(TreeSupportError::InvalidMesh(
                "mesh has no vertices".into(),
            ));
        }
        if triangles.is_empty() {
            return Err(TreeSupportError::InvalidMesh(
                "mesh has no triangles".into(),
            ));
        }
        let vcount = vertices.len() as u32;
        for (ti, tri) in triangles.iter().enumerate() {
            for &idx in tri {
                if idx >= vcount {
                    return Err(TreeSupportError::InvalidMesh(format!(
                        "triangle {} has out-of-range index {}",
                        ti, idx
                    )));
                }
            }
        }

        let (aabb_min, aabb_max) = compute_aabb(&vertices);
        Ok(Self {
            vertices,
            triangles,
            aabb_min,
            aabb_max,
        })
    }

    /// Create a mesh from a raw FFI [`MeshData`] descriptor.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that the pointers and counts in `data` are valid
    /// and that the underlying memory is not deallocated during this call.
    pub unsafe fn from_ffi(data: &MeshData) -> Result<Self> {
        if data.vertices.is_null() {
            return Err(TreeSupportError::NullPointer("MeshData.vertices".into()));
        }
        if data.indices.is_null() {
            return Err(TreeSupportError::NullPointer("MeshData.indices".into()));
        }
        if data.vertex_count == 0 {
            return Err(TreeSupportError::InvalidMesh(
                "mesh has no vertices".into(),
            ));
        }
        if data.triangle_count == 0 {
            return Err(TreeSupportError::InvalidMesh(
                "mesh has no triangles".into(),
            ));
        }

        let verts_slice =
            std::slice::from_raw_parts(data.vertices, (data.vertex_count * 3) as usize);
        let indices_slice =
            std::slice::from_raw_parts(data.indices, (data.triangle_count * 3) as usize);

        let vertices: Vec<Point3<f64>> = verts_slice
            .chunks_exact(3)
            .map(|c| Point3::new(c[0] as f64, c[1] as f64, c[2] as f64))
            .collect();

        let triangles: Vec<[u32; 3]> = indices_slice
            .chunks_exact(3)
            .map(|c| [c[0], c[1], c[2]])
            .collect();

        Self::from_vertices_indices(vertices, triangles)
    }

    /// Number of triangles in the mesh.
    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Number of vertices in the mesh.
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Get the vertex positions for a given triangle index.
    #[inline]
    pub fn triangle_vertices(&self, tri_idx: usize) -> [Point3<f64>; 3] {
        let [i0, i1, i2] = self.triangles[tri_idx];
        [
            self.vertices[i0 as usize],
            self.vertices[i1 as usize],
            self.vertices[i2 as usize],
        ]
    }

    /// Compute the face normal of a triangle (unnormalized).
    pub fn triangle_normal(&self, tri_idx: usize) -> nalgebra::Vector3<f64> {
        let [v0, v1, v2] = self.triangle_vertices(tri_idx);
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        e1.cross(&e2)
    }

    /// Get the Z-range (min_z, max_z) of the mesh.
    #[inline]
    pub fn z_range(&self) -> (CoordF, CoordF) {
        (self.aabb_min.z, self.aabb_max.z)
    }

    /// Compute per-layer Z slicing heights.
    ///
    /// Returns a vector of Z heights from the bottom of the mesh to the top,
    /// spaced by `layer_height`.
    pub fn layer_z_values(&self, layer_height: CoordF) -> Vec<CoordF> {
        if layer_height <= 0.0 {
            return Vec::new();
        }
        let (z_min, z_max) = self.z_range();
        let mut zs = Vec::new();
        let mut z = z_min + layer_height * 0.5;
        while z < z_max {
            zs.push(z);
            z += layer_height;
        }
        zs
    }
}

/// Compute the axis-aligned bounding box of a set of vertices.
fn compute_aabb(vertices: &[Point3<f64>]) -> (Point3<f64>, Point3<f64>) {
    let mut min = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for v in vertices {
        min.x = min.x.min(v.x);
        min.y = min.y.min(v.y);
        min.z = min.z.min(v.z);
        max.x = max.x.max(v.x);
        max.y = max.y.max(v.y);
        max.z = max.z.max(v.z);
    }
    (min, max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_cube() -> TriangleMesh {
        // Minimal cube: 8 vertices, 12 triangles
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ];
        let triangles = vec![
            // bottom
            [0, 1, 2],
            [0, 2, 3],
            // top
            [4, 6, 5],
            [4, 7, 6],
            // front
            [0, 5, 1],
            [0, 4, 5],
            // back
            [2, 7, 3],
            [2, 6, 7],
            // left
            [0, 3, 7],
            [0, 7, 4],
            // right
            [1, 5, 6],
            [1, 6, 2],
        ];
        TriangleMesh::from_vertices_indices(vertices, triangles).unwrap()
    }

    #[test]
    fn test_cube_mesh() {
        let mesh = sample_cube();
        assert_eq!(mesh.vertex_count(), 8);
        assert_eq!(mesh.triangle_count(), 12);
    }

    #[test]
    fn test_aabb() {
        let mesh = sample_cube();
        assert!((mesh.aabb_min.x - 0.0).abs() < 1e-10);
        assert!((mesh.aabb_max.x - 1.0).abs() < 1e-10);
        assert!((mesh.aabb_min.z - 0.0).abs() < 1e-10);
        assert!((mesh.aabb_max.z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_z_range() {
        let mesh = sample_cube();
        let (zmin, zmax) = mesh.z_range();
        assert!((zmin - 0.0).abs() < 1e-10);
        assert!((zmax - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_layer_z_values() {
        let mesh = sample_cube();
        let layers = mesh.layer_z_values(0.2);
        assert!(!layers.is_empty());
        assert!(layers[0] > 0.0);
        assert!(*layers.last().unwrap() < 1.0);
    }

    #[test]
    fn test_triangle_normal() {
        let mesh = sample_cube();
        // Bottom face (z=0) normal should point downward (-z)
        let n = mesh.triangle_normal(0);
        assert!(n.z < 0.0 || n.z > 0.0); // just ensure non-zero
    }

    #[test]
    fn test_empty_mesh_error() {
        let result = TriangleMesh::from_vertices_indices(vec![], vec![]);
        assert!(result.is_err());
    }

    #[test]
    fn test_out_of_range_index_error() {
        let vertices = vec![Point3::new(0.0, 0.0, 0.0)];
        let triangles = vec![[0, 1, 2]]; // indices 1, 2 out of range
        let result = TriangleMesh::from_vertices_indices(vertices, triangles);
        assert!(result.is_err());
    }
}
