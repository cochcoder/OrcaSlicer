//! Interface layer generation (support roof and floor).
//!
//! This module generates the interface layers between the support structure and
//! the model. Interface layers provide a smooth surface for better print quality
//! at the contact points.
//!
//! - **Roof** (top interface): Layers between support body and the model above.
//! - **Floor** (bottom interface): Layers between support body and the model below.
//!
//! Interface layers typically use a denser fill pattern than the support body.

use crate::branch_generation::Branch;
use crate::config::{InterfacePreference, TreeSupportConfig};
use crate::utils::{CoordF, CoordT, Point2D};

/// A single interface layer (roof or floor).
#[derive(Debug, Clone)]
pub struct InterfaceLayer {
    /// Z height of this interface layer.
    pub z: CoordF,
    /// Layer index.
    pub layer_index: usize,
    /// Whether this is a roof (true) or floor (false) layer.
    pub is_roof: bool,
    /// Boundary polygons of the interface area.
    pub polygons: Vec<Vec<Point2D>>,
    /// Fill angle in radians for the interface pattern.
    pub fill_angle: f64,
}

/// Result of interface layer generation.
#[derive(Debug, Clone)]
pub struct InterfaceResult {
    /// Generated roof layers, sorted by layer index.
    pub roof_layers: Vec<InterfaceLayer>,
    /// Generated floor layers, sorted by layer index.
    pub floor_layers: Vec<InterfaceLayer>,
}

/// Generate interface layers (roof and floor) from branches.
///
/// For each branch contact point, generates the configured number of roof layers
/// above and floor layers below, using the branch radius to define the interface
/// area.
///
/// # Arguments
///
/// * `branches` - Generated tree branches.
/// * `config` - Tree support configuration.
/// * `layer_heights` - Z heights for each layer.
///
/// # Examples
///
/// ```
/// use orca_tree_supports::interface_layers::generate_interface_layers;
/// use orca_tree_supports::branch_generation::{Branch, SupportNode};
/// use orca_tree_supports::config::TreeSupportConfig;
/// use orca_tree_supports::utils::Point2D;
///
/// let branches = vec![Branch {
///     nodes: vec![SupportNode::new_contact(0, Point2D::new(0, 0), 1.0, 5, 200_000)],
///     reaches_buildplate: true,
/// }];
/// let config = TreeSupportConfig::default();
/// let layer_heights: Vec<f64> = (0..10).map(|i| i as f64 * 0.2).collect();
/// let result = generate_interface_layers(&branches, &config, &layer_heights);
/// ```
pub fn generate_interface_layers(
    branches: &[Branch],
    config: &TreeSupportConfig,
    layer_heights: &[CoordF],
) -> InterfaceResult {
    let mut roof_layers = Vec::new();
    let mut floor_layers = Vec::new();

    if layer_heights.is_empty() {
        return InterfaceResult {
            roof_layers,
            floor_layers,
        };
    }

    for branch in branches {
        if branch.nodes.is_empty() {
            continue;
        }

        let contact_node = &branch.nodes[0]; // Top node (contact point).

        // Generate roof layers if enabled.
        if config.support_roof_enable {
            let num_roof = config.support_roof_layers as usize;
            let contact_layer = contact_node.layer_index;

            for i in 0..num_roof {
                let roof_layer_idx = contact_layer.saturating_sub(i + 1);
                if roof_layer_idx >= layer_heights.len() {
                    continue;
                }

                let z = layer_heights[roof_layer_idx];
                let radius = contact_node.radius;

                // Create a circular polygon for the interface area.
                let polygon = generate_circle_polygon(
                    &contact_node.position,
                    radius,
                    16, // segments
                );

                // Compute fill angle with rotation per layer (45° base, rotating 30° per layer).
                let base_angle = std::f64::consts::PI / 4.0;
                let fill_angle = base_angle + (i as f64) * std::f64::consts::PI / 6.0;

                roof_layers.push(InterfaceLayer {
                    z,
                    layer_index: roof_layer_idx,
                    is_roof: true,
                    polygons: vec![polygon],
                    fill_angle,
                });
            }
        }

        // Generate floor layers if enabled.
        if config.support_floor_enable {
            let num_floor = config.support_floor_layers as usize;
            // Floor is at the bottom of the branch.
            if let Some(base_node) = branch.nodes.last() {
                let base_layer = base_node.layer_index;

                for i in 0..num_floor {
                    let floor_layer_idx = base_layer + i;
                    if floor_layer_idx >= layer_heights.len() {
                        continue;
                    }

                    let z = layer_heights[floor_layer_idx];
                    let radius = base_node.radius;

                    let polygon = generate_circle_polygon(
                        &base_node.position,
                        radius,
                        16,
                    );

                    let fill_angle = std::f64::consts::PI / 4.0 + (i as f64) * std::f64::consts::PI / 6.0;

                    floor_layers.push(InterfaceLayer {
                        z,
                        layer_index: floor_layer_idx,
                        is_roof: false,
                        polygons: vec![polygon],
                        fill_angle,
                    });
                }
            }
        }
    }

    // Sort by layer index.
    roof_layers.sort_by_key(|l| l.layer_index);
    floor_layers.sort_by_key(|l| l.layer_index);

    InterfaceResult {
        roof_layers,
        floor_layers,
    }
}

/// Resolve overlap between interface areas and support body areas.
///
/// Applies the configured [`InterfacePreference`] to determine which takes
/// priority when interface and support body overlap.
pub fn resolve_interface_overlap(
    _interface: &mut InterfaceResult,
    preference: InterfacePreference,
) {
    match preference {
        InterfacePreference::InterfaceAreaOverwritesSupport
        | InterfacePreference::InterfaceLinesOverwriteSupport => {
            // Interface takes priority; no changes needed to interface polygons.
        }
        InterfacePreference::SupportAreaOverwritesInterface
        | InterfacePreference::SupportLinesOverwriteInterface => {
            // In a full implementation, this would subtract support areas from
            // interface areas. For now, we leave interface as-is.
        }
        InterfacePreference::Nothing => {
            // No overlap resolution needed.
        }
    }
}

/// Generate a circular polygon approximation.
///
/// Creates a polygon with `segments` vertices approximating a circle centered
/// at `center` with the given `radius`.
fn generate_circle_polygon(center: &Point2D, radius: CoordT, segments: usize) -> Vec<Point2D> {
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

    fn make_test_branch() -> Branch {
        let contact = SupportNode::new_contact(
            0,
            Point2D::new(5_000_000, 5_000_000),
            1.0,
            5,
            200_000,
        );
        Branch {
            nodes: vec![contact],
            reaches_buildplate: true,
        }
    }

    #[test]
    fn test_generate_interface_no_roof_no_floor() {
        let branches = vec![make_test_branch()];
        let config = TreeSupportConfig::default(); // roof/floor disabled by default
        let layer_heights: Vec<f64> = (0..10).map(|i| i as f64 * 0.2).collect();
        let result = generate_interface_layers(&branches, &config, &layer_heights);
        assert!(result.roof_layers.is_empty());
        assert!(result.floor_layers.is_empty());
    }

    #[test]
    fn test_generate_interface_with_roof() {
        let branches = vec![make_test_branch()];
        let mut config = TreeSupportConfig::default();
        config.support_roof_enable = true;
        config.support_roof_layers = 2;
        let layer_heights: Vec<f64> = (0..10).map(|i| i as f64 * 0.2).collect();
        let result = generate_interface_layers(&branches, &config, &layer_heights);
        assert!(!result.roof_layers.is_empty());
        assert!(result.roof_layers.len() <= 2);
    }

    #[test]
    fn test_generate_interface_with_floor() {
        let branches = vec![make_test_branch()];
        let mut config = TreeSupportConfig::default();
        config.support_floor_enable = true;
        config.support_floor_layers = 2;
        let layer_heights: Vec<f64> = (0..10).map(|i| i as f64 * 0.2).collect();
        let result = generate_interface_layers(&branches, &config, &layer_heights);
        assert!(!result.floor_layers.is_empty());
    }

    #[test]
    fn test_generate_circle_polygon() {
        let center = Point2D::new(0, 0);
        let polygon = generate_circle_polygon(&center, 1_000_000, 16);
        assert_eq!(polygon.len(), 16);
        // All points should be approximately 1mm from center.
        for p in &polygon {
            let dist = center.distance_to(p);
            assert!((dist - 1_000_000.0).abs() < 100.0);
        }
    }

    #[test]
    fn test_empty_branches() {
        let branches: Vec<Branch> = Vec::new();
        let config = TreeSupportConfig::default();
        let layer_heights: Vec<f64> = (0..10).map(|i| i as f64 * 0.2).collect();
        let result = generate_interface_layers(&branches, &config, &layer_heights);
        assert!(result.roof_layers.is_empty());
        assert!(result.floor_layers.is_empty());
    }
}
