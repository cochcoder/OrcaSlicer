#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

/// How interface areas interact with support areas.
///
/// Mirrors the C++ `InterfacePreference` enum.
enum class InterfacePreference {
  /// Interface areas overwrite support areas.
  InterfaceAreaOverwritesSupport = 0,
  /// Support areas overwrite interface areas.
  SupportAreaOverwritesInterface = 1,
  /// Interface lines overwrite support areas.
  InterfaceLinesOverwriteSupport = 2,
  /// Support lines overwrite interface areas.
  SupportLinesOverwriteInterface = 3,
  /// No preference.
  Nothing = 4,
};

/// Type of support roof/interface pattern.
///
/// Mirrors a subset of `SupportMaterialInterfacePattern` from C++.
enum class SupportInterfacePattern {
  /// Automatic selection.
  Auto = 0,
  /// Rectilinear pattern.
  Rectilinear = 1,
  /// Concentric pattern.
  Concentric = 2,
  /// Grid pattern.
  Grid = 3,
};

/// Type of support pattern.
///
/// Mirrors a subset of `SupportMaterialPattern` from C++.
enum class SupportPattern {
  /// Rectilinear infill pattern.
  Rectilinear = 0,
  /// Grid infill pattern.
  Grid = 1,
  /// Honeycomb infill pattern.
  Honeycomb = 2,
  /// Lightning infill pattern (tree-optimized).
  Lightning = 3,
};

/// Opaque handle to a tree support generation session.
///
/// Created by [`orca_tree_support_create`] and destroyed by
/// [`orca_tree_support_destroy_handle`]. Do not access the internal fields
/// directly from C/C++ code.
struct TreeSupportHandle;

/// Coordinate type matching the C++ `coord_t` (scaled integer coordinates).
///
/// In OrcaSlicer, 1 unit = 1 nanometre (1 mm = 1_000_000 units) when using
/// the default `SCALING_FACTOR` of `1e-6`.
using CoordT = int64_t;

/// Complete tree support configuration.
///
/// All distance/size fields are in scaled coordinates (1 unit = 1 nm by default).
/// Angles are in radians.
///
/// This struct is `#[repr(C)]` so it can be passed directly from C/C++ code via FFI.
///
/// # Examples
///
/// ```
/// use orca_tree_supports::config::TreeSupportConfig;
/// let cfg = TreeSupportConfig::default();
/// assert!(cfg.support_tree_angle > 0.0);
/// ```
struct TreeSupportConfig {
  /// Layer height in scaled coordinates.
  CoordT layer_height;
  /// Resolution for polygon simplification in scaled coordinates.
  CoordT resolution;
  /// Minimum feature size in scaled coordinates.
  CoordT min_feature_size;
  /// Maximum overhang angle (radians) before support is generated.
  double support_angle;
  /// Line width of support structures in scaled coordinates.
  CoordT support_line_width;
  /// Line width of support roof in scaled coordinates.
  CoordT support_roof_line_width;
  /// Whether support floor (bottom interface) is enabled.
  bool support_bottom_enable;
  /// Height of support floor in scaled coordinates.
  CoordT support_bottom_height;
  /// Whether support is limited to touching the build plate only.
  bool support_material_buildplate_only;
  /// XY distance between support and model in scaled coordinates.
  CoordT support_xy_distance;
  /// XY distance for the first layer in scaled coordinates.
  CoordT support_xy_distance_first_layer;
  /// XY distance at overhang boundaries in scaled coordinates.
  CoordT support_xy_distance_overhang;
  /// Top Z gap between model and support in scaled coordinates.
  CoordT support_top_distance;
  /// Bottom Z gap between support and model/plate in scaled coordinates.
  CoordT support_bottom_distance;
  /// Interface skip height in scaled coordinates.
  CoordT support_interface_skip_height;
  /// Whether support roof is enabled.
  bool support_roof_enable;
  /// Number of support roof layers.
  int32_t support_roof_layers;
  /// Whether support floor is enabled.
  bool support_floor_enable;
  /// Number of support floor layers.
  int32_t support_floor_layers;
  /// Minimum roof area (scaled² units).
  double minimum_roof_area;
  /// Support roof interface pattern.
  SupportInterfacePattern roof_pattern;
  /// Support body pattern.
  SupportPattern support_pattern;
  /// Line spacing for support body in scaled coordinates.
  CoordT support_line_spacing;
  /// Bottom offset for support in scaled coordinates.
  CoordT support_bottom_offset;
  /// Number of support wall loops.
  int32_t support_wall_count;
  /// Roof line distance in scaled coordinates.
  CoordT support_roof_line_distance;
  /// Minimum support area in scaled coordinates.
  CoordT minimum_support_area;
  /// Minimum bottom area in scaled coordinates.
  CoordT minimum_bottom_area;
  /// Support offset in scaled coordinates.
  CoordT support_offset;
  /// Maximum branch angle when avoiding the model (radians).
  double support_tree_angle;
  /// Preferred branch angle without model avoidance (radians).
  double support_tree_angle_slow;
  /// Branch diameter at the base in scaled coordinates.
  CoordT support_tree_branch_diameter;
  /// How much the branch thickens toward the base (radians).
  double support_tree_branch_diameter_angle;
  /// Distance between branch tips in scaled coordinates.
  CoordT support_tree_branch_distance;
  /// Build plate contact diameter in scaled coordinates.
  CoordT support_tree_bp_diameter;
  /// Tip density percentage (0–100).
  double support_tree_top_rate;
  /// Tip diameter in scaled coordinates.
  CoordT support_tree_tip_diameter;
  /// Maximum diameter increase by merges when support rests on model, in
  /// scaled coordinates.
  CoordT support_tree_max_diameter_increase_by_merges_when_support_to_model;
  /// Minimum height to model for tree support in scaled coordinates.
  CoordT support_tree_min_height_to_model;
  /// How interface areas interact with support body.
  InterfacePreference interface_preference;
  /// Whether support can rest on the model surface.
  bool support_rests_on_model;
};

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
struct MeshData {
  /// Pointer to packed vertex coordinates: [x0, y0, z0, x1, y1, z1, …].
  const float *vertices;
  /// Number of vertices (each vertex is 3 floats).
  uint32_t vertex_count;
  /// Pointer to packed triangle indices: [i0, j0, k0, i1, j1, k1, …].
  const uint32_t *indices;
  /// Number of triangles (each triangle is 3 indices).
  uint32_t triangle_count;
};

/// Floating-point coordinate type matching C++ `coordf_t`.
using CoordF = double;

/// A 2D point in scaled integer coordinates, matching C++ `Point`.
struct Point2D {
  /// X coordinate in scaled units.
  CoordT x;
  /// Y coordinate in scaled units.
  CoordT y;
};

/// A single layer of support geometry in the final output.
struct SupportLayerOutput {
  /// Z height of this layer.
  CoordF z;
  /// Layer index.
  uint32_t layer_index;
  /// Number of polygons in this layer.
  uint32_t polygon_count;
  /// Pointer to polygon data (owned by the output).
  ///
  /// Each polygon is represented as a contiguous sequence of `Point2D`
  /// values, with `polygon_sizes` indicating the number of points per polygon.
  ///
  /// # Safety
  ///
  /// This pointer is valid only for the lifetime of the containing
  /// [`SupportOutput`]. Do not free it separately.
  const Point2D *polygon_points;
  /// Total number of points across all polygons in this layer.
  uint32_t total_point_count;
  /// Pointer to an array of polygon sizes (number of points per polygon).
  const uint32_t *polygon_sizes;
};

/// Complete support output from tree support generation.
///
/// This struct is `#[repr(C)]` for FFI compatibility. It owns all the data
/// and must be freed by calling [`orca_tree_support_destroy_output`](crate::ffi::orca_tree_support_destroy_output).
struct SupportOutput {
  /// Number of layers with support geometry.
  uint32_t layer_count;
  /// Pointer to the array of layer outputs.
  const SupportLayerOutput *layers;
  /// Total number of branches generated.
  uint32_t branch_count;
  /// Whether generation completed successfully.
  bool success;
};

extern "C" {

/// Create a new tree support generation handle.
///
/// # Safety
///
/// - `config` must be a valid, non-null pointer to a [`TreeSupportConfig`].
/// - `mesh` must be a valid, non-null pointer to a [`MeshData`] with valid
///   vertex and index pointers and counts.
/// - The mesh data must remain valid until the handle is destroyed.
///
/// Returns a null pointer on failure (e.g., invalid config or mesh).
TreeSupportHandle *orca_tree_support_create(const TreeSupportConfig *config, const MeshData *mesh);

/// Generate tree support structures.
///
/// Runs the full tree support generation pipeline and returns the output.
///
/// # Safety
///
/// - `handle` must be a valid, non-null pointer returned by
///   [`orca_tree_support_create`].
/// - The handle must not have been previously destroyed.
///
/// Returns a null pointer on failure.
SupportOutput *orca_tree_support_generate(TreeSupportHandle *handle);

/// Destroy a tree support handle and free its resources.
///
/// # Safety
///
/// - `handle` must be a valid pointer returned by [`orca_tree_support_create`],
///   or null (in which case this function is a no-op).
/// - The handle must not be used after this call.
void orca_tree_support_destroy_handle(TreeSupportHandle *handle);

/// Destroy a support output and free its resources.
///
/// # Safety
///
/// - `output` must be a valid pointer returned by [`orca_tree_support_generate`],
///   or null (in which case this function is a no-op).
/// - The output must not be used after this call.
void orca_tree_support_destroy_output(SupportOutput *output);

}  // extern "C"
