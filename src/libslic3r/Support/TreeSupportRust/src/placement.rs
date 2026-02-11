//! Final geometry placement and output generation.
//!
//! This module converts tree branches and interface layers into the final
//! per-layer support polygons that can be sliced and filled with toolpaths.
//!
//! The output format ([`SupportOutput`]) is FFI-compatible and can be passed
//! back to the C++ caller.

use crate::branch_generation::BranchGenerationResult;
use crate::config::TreeSupportConfig;
use crate::interface_layers::InterfaceResult;
use crate::utils::{CoordF, CoordT, Point2D};

/// A single layer of support geometry in the final output.
#[derive(Debug, Clone)]
#[repr(C)]
pub struct SupportLayerOutput {
    /// Z height of this layer.
    pub z: CoordF,
    /// Layer index.
    pub layer_index: u32,
    /// Number of polygons in this layer.
    pub polygon_count: u32,
    /// Pointer to polygon data (owned by the output).
    ///
    /// Each polygon is represented as a contiguous sequence of `Point2D`
    /// values, with `polygon_sizes` indicating the number of points per polygon.
    ///
    /// # Safety
    ///
    /// This pointer is valid only for the lifetime of the containing
    /// [`SupportOutput`]. Do not free it separately.
    pub polygon_points: *const Point2D,
    /// Total number of points across all polygons in this layer.
    pub total_point_count: u32,
    /// Pointer to an array of polygon sizes (number of points per polygon).
    pub polygon_sizes: *const u32,
}

/// Complete support output from tree support generation.
///
/// This struct is `#[repr(C)]` for FFI compatibility. It owns all the data
/// and must be freed by calling [`orca_tree_support_destroy_output`](crate::ffi::orca_tree_support_destroy_output).
#[derive(Debug)]
#[repr(C)]
pub struct SupportOutput {
    /// Number of layers with support geometry.
    pub layer_count: u32,
    /// Pointer to the array of layer outputs.
    pub layers: *const SupportLayerOutput,
    /// Total number of branches generated.
    pub branch_count: u32,
    /// Whether generation completed successfully.
    pub success: bool,
}

/// Owned version of support output (used internally before converting to FFI).
#[derive(Debug, Clone)]
pub struct SupportOutputOwned {
    /// Per-layer support geometry.
    pub layers: Vec<SupportLayerOwned>,
    /// Number of branches.
    pub branch_count: usize,
}

/// Owned version of a single support layer.
#[derive(Debug, Clone)]
pub struct SupportLayerOwned {
    /// Z height.
    pub z: CoordF,
    /// Layer index.
    pub layer_index: usize,
    /// Polygons as sequences of points.
    pub polygons: Vec<Vec<Point2D>>,
}

/// Convert branches and interface layers into the final support output.
///
/// This function takes the branch tree and interface layers and produces
/// per-layer polygon outlines that define the support geometry.
///
/// Each branch node is converted to a circle polygon at its layer, and
/// interface layers are merged into the output.
///
/// # Arguments
///
/// * `branches` - Generated tree branches from the branch generation phase.
/// * `interface` - Interface layer data (roof/floor).
/// * `config` - Tree support configuration.
/// * `layer_heights` - Z heights for each layer.
///
/// # Examples
///
/// ```
/// use orca_tree_supports::placement::{generate_support_output, SupportOutputOwned};
/// use orca_tree_supports::branch_generation::{Branch, BranchGenerationResult, SupportNode};
/// use orca_tree_supports::interface_layers::InterfaceResult;
/// use orca_tree_supports::config::TreeSupportConfig;
/// use orca_tree_supports::utils::Point2D;
///
/// let branches = BranchGenerationResult {
///     branches: vec![Branch {
///         nodes: vec![SupportNode::new_contact(0, Point2D::new(0, 0), 0.5, 2, 200_000)],
///         reaches_buildplate: true,
///     }],
///     total_nodes: 1,
/// };
/// let interface = InterfaceResult {
///     roof_layers: vec![],
///     floor_layers: vec![],
/// };
/// let config = TreeSupportConfig::default();
/// let layer_heights = vec![0.0, 0.2, 0.4, 0.6];
/// let output = generate_support_output(&branches, &interface, &config, &layer_heights);
/// assert!(output.branch_count > 0);
/// ```
pub fn generate_support_output(
    branches: &BranchGenerationResult,
    interface: &InterfaceResult,
    _config: &TreeSupportConfig,
    layer_heights: &[CoordF],
) -> SupportOutputOwned {
    let num_layers = layer_heights.len();
    let mut layers: Vec<SupportLayerOwned> = (0..num_layers)
        .map(|i| SupportLayerOwned {
            z: layer_heights[i],
            layer_index: i,
            polygons: Vec::new(),
        })
        .collect();

    // Convert branch nodes to circle polygons at each layer.
    for branch in &branches.branches {
        for node in &branch.nodes {
            if node.layer_index < num_layers {
                let polygon = draw_circle(&node.position, node.radius, 16);
                layers[node.layer_index].polygons.push(polygon);
            }
        }
    }

    // Add interface (roof/floor) polygons.
    for roof in &interface.roof_layers {
        if roof.layer_index < num_layers {
            layers[roof.layer_index]
                .polygons
                .extend(roof.polygons.clone());
        }
    }
    for floor in &interface.floor_layers {
        if floor.layer_index < num_layers {
            layers[floor.layer_index]
                .polygons
                .extend(floor.polygons.clone());
        }
    }

    // Remove empty layers for a compact output.
    layers.retain(|l| !l.polygons.is_empty());

    SupportOutputOwned {
        layers,
        branch_count: branches.branches.len(),
    }
}

/// Draw a circle polygon with the given center, radius, and number of segments.
fn draw_circle(center: &Point2D, radius: CoordT, segments: usize) -> Vec<Point2D> {
    let mut points = Vec::with_capacity(segments);
    for i in 0..segments {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (segments as f64);
        let x = center.x + (radius as f64 * angle.cos()) as CoordT;
        let y = center.y + (radius as f64 * angle.sin()) as CoordT;
        points.push(Point2D::new(x, y));
    }
    points
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::branch_generation::{Branch, SupportNode};
    use crate::interface_layers::InterfaceResult;

    #[test]
    fn test_generate_support_output_basic() {
        let node = SupportNode::new_contact(0, Point2D::new(5_000_000, 5_000_000), 0.4, 2, 200_000);
        let branches = BranchGenerationResult {
            branches: vec![Branch {
                nodes: vec![node],
                reaches_buildplate: true,
            }],
            total_nodes: 1,
        };
        let interface = InterfaceResult {
            roof_layers: vec![],
            floor_layers: vec![],
        };
        let config = TreeSupportConfig::default();
        let layer_heights = vec![0.0, 0.2, 0.4, 0.6];
        let output = generate_support_output(&branches, &interface, &config, &layer_heights);
        assert_eq!(output.branch_count, 1);
        assert!(!output.layers.is_empty());
    }

    #[test]
    fn test_generate_support_output_empty() {
        let branches = BranchGenerationResult {
            branches: vec![],
            total_nodes: 0,
        };
        let interface = InterfaceResult {
            roof_layers: vec![],
            floor_layers: vec![],
        };
        let config = TreeSupportConfig::default();
        let layer_heights = vec![0.0, 0.2, 0.4];
        let output = generate_support_output(&branches, &interface, &config, &layer_heights);
        assert_eq!(output.branch_count, 0);
        assert!(output.layers.is_empty());
    }

    #[test]
    fn test_draw_circle() {
        let center = Point2D::new(0, 0);
        let polygon = draw_circle(&center, 1_000_000, 8);
        assert_eq!(polygon.len(), 8);
    }
}
