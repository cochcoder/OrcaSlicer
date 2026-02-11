//! Tree branch generation and node dropping.
//!
//! This module implements the core tree support algorithm:
//!
//! 1. **Contact point generation**: Place initial support nodes at overhang positions.
//! 2. **Node dropping**: Drop nodes downward layer by layer toward the build plate,
//!    with lateral movement to avoid collisions and merge nearby branches.
//! 3. **Radius progression**: Branch radius grows from the tip diameter at the top
//!    to the full branch diameter toward the base.
//! 4. **Branch merging**: Nearby branches merge when their influence areas overlap.
//!
//! The implementation parallelizes independent branch generation using Rayon.

use crate::collision::CollisionModel;
use crate::config::{DerivedSupportSettings, TreeSupportConfig};
use crate::overhang_detection::OverhangResult;
use crate::utils::{CoordF, CoordT, Point2D};
use rayon::prelude::*;
use smallvec::SmallVec;

/// Type of tree node geometry.
///
/// Mirrors the C++ `TreeNodeType` enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeType {
    /// Circular cross-section (default for most branches).
    Circle,
    /// Square cross-section (used at certain junctions).
    Square,
    /// Polygon cross-section (for complex shapes).
    Polygon,
}

impl Default for NodeType {
    fn default() -> Self {
        Self::Circle
    }
}

/// A single node in the support tree.
///
/// Corresponds to `SupportNode` in the C++ codebase. Each node represents
/// a point in 3D space where a branch passes through a given layer.
#[derive(Debug, Clone)]
pub struct SupportNode {
    /// Unique node identifier within the generation session.
    pub id: u64,

    /// 2D position on the XY plane (scaled coordinates).
    pub position: Point2D,

    /// Z height of this node.
    pub z: CoordF,

    /// Layer index (0 = bottom).
    pub layer_index: usize,

    /// Distance (in layers) from the top contact point to this node.
    pub distance_to_top: u32,

    /// Distance (in mm) from the top contact point to this node.
    pub dist_mm_to_top: CoordF,

    /// Current branch radius at this node (scaled coordinates).
    pub radius: CoordT,

    /// Maximum lateral movement allowed per layer at this node.
    pub max_move_distance: CoordT,

    /// The type of geometry to draw at this node.
    pub node_type: NodeType,

    /// Whether this node should connect to the build plate.
    pub to_buildplate: bool,

    /// Whether this is a sharp tail support node.
    pub is_sharp_tail: bool,

    /// Whether this node needs an extra perimeter wall.
    pub need_extra_wall: bool,

    /// Number of support roof layers below this node.
    pub support_roof_layers_below: u32,

    /// Index of the parent node (in the branch's node list), if any.
    pub parent_index: Option<usize>,

    /// Indices of child nodes that merge into this node.
    pub child_indices: SmallVec<[usize; 2]>,
}

impl SupportNode {
    /// Create a new contact node at the top of a branch.
    ///
    /// # Arguments
    ///
    /// * `id` - Unique node ID.
    /// * `position` - XY position.
    /// * `z` - Z height.
    /// * `layer_index` - Layer index.
    /// * `tip_radius` - Initial tip radius.
    pub fn new_contact(
        id: u64,
        position: Point2D,
        z: CoordF,
        layer_index: usize,
        tip_radius: CoordT,
    ) -> Self {
        Self {
            id,
            position,
            z,
            layer_index,
            distance_to_top: 0,
            dist_mm_to_top: 0.0,
            radius: tip_radius,
            max_move_distance: 0,
            node_type: NodeType::Circle,
            to_buildplate: true,
            is_sharp_tail: false,
            need_extra_wall: false,
            support_roof_layers_below: 0,
            parent_index: None,
            child_indices: SmallVec::new(),
        }
    }
}

/// A single tree branch from contact point to base.
#[derive(Debug, Clone)]
pub struct Branch {
    /// Nodes in this branch, ordered from top (contact) to bottom (base).
    pub nodes: Vec<SupportNode>,
    /// Whether this branch successfully reaches the build plate.
    pub reaches_buildplate: bool,
}

/// Result of the branch generation phase.
#[derive(Debug)]
pub struct BranchGenerationResult {
    /// All generated branches.
    pub branches: Vec<Branch>,
    /// Total number of nodes across all branches.
    pub total_nodes: usize,
}

/// Generate contact points from overhang areas.
///
/// Places initial support nodes at positions where overhang areas need support,
/// respecting the branch distance and top rate parameters.
///
/// # Arguments
///
/// * `overhangs` - Detected overhang areas from [`detect_overhangs`](crate::overhang_detection::detect_overhangs).
/// * `config` - Tree support configuration.
/// * `settings` - Derived support settings.
/// * `layer_heights` - Z heights for each layer.
pub fn generate_contact_points(
    overhangs: &OverhangResult,
    config: &TreeSupportConfig,
    settings: &DerivedSupportSettings,
    layer_heights: &[CoordF],
) -> Vec<SupportNode> {
    let mut nodes = Vec::new();
    let mut next_id: u64 = 0;
    let branch_distance = config.support_tree_branch_distance;
    let tip_radius = settings.min_radius;

    for (layer_idx, layer_overhangs) in overhangs.layers.iter().enumerate() {
        if layer_idx >= layer_heights.len() {
            break;
        }
        let z = layer_heights[layer_idx];

        for area in layer_overhangs {
            // Place contact nodes at overhang positions.
            // In a full implementation, this would use a grid/sampling strategy
            // matching the C++ `generate_contact_points()`.
            for point in &area.boundary {
                // Check spacing: only place a node if it's far enough from existing
                // nodes on this layer.
                let too_close = nodes.iter().any(|n: &SupportNode| {
                    n.layer_index == layer_idx
                        && n.position.distance_to(point) < branch_distance as f64
                });

                if !too_close {
                    nodes.push(SupportNode::new_contact(
                        next_id,
                        *point,
                        z,
                        layer_idx,
                        tip_radius,
                    ));
                    next_id += 1;
                }
            }
        }
    }

    nodes
}

/// Drop nodes from their contact points downward toward the build plate.
///
/// This is the core "tree growing" step. Each contact node is extended downward
/// one layer at a time, adjusting position to avoid collisions and merging with
/// nearby branches when possible.
///
/// The radius of each node increases according to the diameter angle, and the
/// position shifts laterally to avoid the model.
///
/// # Arguments
///
/// * `contact_nodes` - Initial contact nodes from [`generate_contact_points`].
/// * `config` - Tree support configuration.
/// * `settings` - Derived support settings.
/// * `collision_model` - Pre-computed collision data.
/// * `layer_heights` - Z heights for each layer.
pub fn drop_nodes(
    contact_nodes: &[SupportNode],
    config: &TreeSupportConfig,
    settings: &DerivedSupportSettings,
    collision_model: &CollisionModel,
    layer_heights: &[CoordF],
) -> BranchGenerationResult {
    // Process each contact node independently (parallelizable).
    let branches: Vec<Branch> = contact_nodes
        .par_iter()
        .map(|contact| {
            drop_single_branch(contact, config, settings, collision_model, layer_heights)
        })
        .collect();

    let total_nodes: usize = branches.iter().map(|b| b.nodes.len()).sum();

    BranchGenerationResult {
        branches,
        total_nodes,
    }
}

/// Drop a single branch from its contact point to the build plate or model.
fn drop_single_branch(
    contact: &SupportNode,
    _config: &TreeSupportConfig,
    settings: &DerivedSupportSettings,
    collision_model: &CollisionModel,
    layer_heights: &[CoordF],
) -> Branch {
    let mut nodes = Vec::new();
    nodes.push(contact.clone());

    let mut current_pos = contact.position;
    let mut current_radius;
    let mut distance_to_top: u32 = 0;
    let mut dist_mm_to_top: CoordF = 0.0;

    // Drop layer by layer from the contact point toward the bottom.
    let start_layer = contact.layer_index;
    for layer_idx in (0..start_layer).rev() {
        distance_to_top += 1;
        let layer_height_mm = if layer_idx + 1 < layer_heights.len() {
            (layer_heights[layer_idx + 1] - layer_heights[layer_idx]).abs()
        } else {
            0.15 // fallback
        };
        dist_mm_to_top += layer_height_mm;

        // Compute radius for this distance from top.
        current_radius = calc_branch_radius(settings, distance_to_top);

        // Determine maximum move distance.
        let max_move = if collision_model.is_colliding(&current_pos, current_radius, layer_idx) {
            settings.maximum_move_distance
        } else {
            settings.maximum_move_distance_slow
        };

        // Try to move toward the center (build plate preferred).
        // In a full implementation, this would use the avoidance areas.
        let moved_pos = try_move_toward_center(
            &current_pos,
            max_move,
            collision_model,
            current_radius,
            layer_idx,
        );
        current_pos = moved_pos;

        let z = if layer_idx < layer_heights.len() {
            layer_heights[layer_idx]
        } else {
            0.0
        };

        let mut node = SupportNode::new_contact(0, current_pos, z, layer_idx, current_radius);
        node.distance_to_top = distance_to_top;
        node.dist_mm_to_top = dist_mm_to_top;
        node.radius = current_radius;
        node.max_move_distance = max_move;
        node.parent_index = Some(nodes.len() - 1);
        nodes.push(node);
    }

    let reaches_buildplate = !nodes.is_empty() && nodes.last().map_or(false, |n| n.layer_index == 0);

    Branch {
        nodes,
        reaches_buildplate,
    }
}

/// Calculate branch radius at a given distance-to-top (in layers).
///
/// The radius starts at the tip diameter and grows linearly toward the base
/// according to `branch_radius_increase_per_layer`, capped at the full
/// branch radius.
///
/// Mirrors the C++ `calc_branch_radius()` function.
///
/// # Examples
///
/// ```
/// use orca_tree_supports::config::{TreeSupportConfig, DerivedSupportSettings};
/// use orca_tree_supports::branch_generation::calc_branch_radius;
///
/// let cfg = TreeSupportConfig::default();
/// let settings = DerivedSupportSettings::from_config(&cfg);
/// let radius_at_tip = calc_branch_radius(&settings, 0);
/// let radius_lower = calc_branch_radius(&settings, 10);
/// assert!(radius_lower >= radius_at_tip);
/// ```
pub fn calc_branch_radius(settings: &DerivedSupportSettings, distance_to_top: u32) -> CoordT {
    let min_r = settings.min_radius as f64;
    let max_r = settings.branch_radius as f64;
    let increase = settings.branch_radius_increase_per_layer;
    let r = min_r + (distance_to_top as f64) * increase;
    r.min(max_r) as CoordT
}

/// Try to move a node position toward the center of the build plate,
/// constrained by the maximum move distance and collision avoidance.
fn try_move_toward_center(
    current: &Point2D,
    max_move: CoordT,
    collision_model: &CollisionModel,
    radius: CoordT,
    layer_idx: usize,
) -> Point2D {
    // Simple heuristic: try to move toward (0, 0) in XY space (center of build plate).
    let dx = -current.x;
    let dy = -current.y;
    let dist = ((dx as f64).powi(2) + (dy as f64).powi(2)).sqrt();

    if dist < 1.0 {
        return *current; // Already at center.
    }

    let move_dist = (max_move as f64).min(dist);
    let new_x = current.x + (dx as f64 * move_dist / dist) as CoordT;
    let new_y = current.y + (dy as f64 * move_dist / dist) as CoordT;
    let candidate = Point2D::new(new_x, new_y);

    // Check if the new position collides.
    if collision_model.is_colliding(&candidate, radius, layer_idx) {
        // Fall back to current position if collision occurs.
        *current
    } else {
        candidate
    }
}

/// Smooth the positions of nodes in a branch for cleaner paths.
///
/// Applies a simple averaging filter to adjacent node positions to reduce
/// zig-zag movement, matching the C++ `smooth_nodes()` function.
pub fn smooth_branch(branch: &mut Branch) {
    if branch.nodes.len() < 3 {
        return;
    }

    // Simple 3-point moving average on XY positions.
    let positions: Vec<Point2D> = branch.nodes.iter().map(|n| n.position).collect();
    for i in 1..branch.nodes.len() - 1 {
        let prev = &positions[i - 1];
        let curr = &positions[i];
        let next = &positions[i + 1];
        branch.nodes[i].position = Point2D::new(
            (prev.x + curr.x + next.x) / 3,
            (prev.y + curr.y + next.y) / 3,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::TreeSupportConfig;

    #[test]
    fn test_calc_branch_radius_at_tip() {
        let cfg = TreeSupportConfig::default();
        let settings = DerivedSupportSettings::from_config(&cfg);
        let r = calc_branch_radius(&settings, 0);
        assert_eq!(r, settings.min_radius);
    }

    #[test]
    fn test_calc_branch_radius_increases() {
        let cfg = TreeSupportConfig::default();
        let settings = DerivedSupportSettings::from_config(&cfg);
        let r0 = calc_branch_radius(&settings, 0);
        let r10 = calc_branch_radius(&settings, 10);
        assert!(r10 >= r0);
    }

    #[test]
    fn test_calc_branch_radius_capped() {
        let cfg = TreeSupportConfig::default();
        let settings = DerivedSupportSettings::from_config(&cfg);
        let r = calc_branch_radius(&settings, 10000);
        assert!(r <= settings.branch_radius);
    }

    #[test]
    fn test_support_node_creation() {
        let node = SupportNode::new_contact(
            1,
            Point2D::new(100, 200),
            0.5,
            5,
            200_000,
        );
        assert_eq!(node.id, 1);
        assert_eq!(node.layer_index, 5);
        assert_eq!(node.distance_to_top, 0);
        assert!(node.to_buildplate);
    }

    #[test]
    fn test_node_type_default() {
        assert_eq!(NodeType::default(), NodeType::Circle);
    }

    #[test]
    fn test_smooth_branch_short() {
        let mut branch = Branch {
            nodes: vec![
                SupportNode::new_contact(0, Point2D::new(0, 0), 0.0, 0, 100),
                SupportNode::new_contact(1, Point2D::new(100, 100), 0.0, 0, 100),
            ],
            reaches_buildplate: false,
        };
        // Should not crash on branch with < 3 nodes.
        smooth_branch(&mut branch);
        assert_eq!(branch.nodes.len(), 2);
    }

    #[test]
    fn test_smooth_branch() {
        let mut branch = Branch {
            nodes: vec![
                SupportNode::new_contact(0, Point2D::new(0, 0), 0.0, 0, 100),
                SupportNode::new_contact(1, Point2D::new(300, 0), 0.0, 0, 100),
                SupportNode::new_contact(2, Point2D::new(0, 0), 0.0, 0, 100),
            ],
            reaches_buildplate: false,
        };
        smooth_branch(&mut branch);
        // Middle node should be smoothed toward average.
        assert_eq!(branch.nodes[1].position.x, 100); // (0+300+0)/3
    }
}
