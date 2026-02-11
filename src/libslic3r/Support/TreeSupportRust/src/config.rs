//! Configuration parameters for tree support generation.
//!
//! This module contains all tunable parameters that control tree support behaviour,
//! mirroring the C++ `TreeSupportMeshGroupSettings` and `TreeSupportSettings` structs.
//!
//! The [`TreeSupportConfig`] struct is the primary public type and is `#[repr(C)]` for
//! FFI compatibility.

use crate::utils::{scaled, CoordT};
use std::f64::consts::PI;

/// How interface areas interact with support areas.
///
/// Mirrors the C++ `InterfacePreference` enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(C)]
pub enum InterfacePreference {
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
}

/// Type of support pattern.
///
/// Mirrors a subset of `SupportMaterialPattern` from C++.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(C)]
pub enum SupportPattern {
    /// Rectilinear infill pattern.
    Rectilinear = 0,
    /// Grid infill pattern.
    Grid = 1,
    /// Honeycomb infill pattern.
    Honeycomb = 2,
    /// Lightning infill pattern (tree-optimized).
    Lightning = 3,
}

/// Type of support roof/interface pattern.
///
/// Mirrors a subset of `SupportMaterialInterfacePattern` from C++.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(C)]
pub enum SupportInterfacePattern {
    /// Automatic selection.
    Auto = 0,
    /// Rectilinear pattern.
    Rectilinear = 1,
    /// Concentric pattern.
    Concentric = 2,
    /// Grid pattern.
    Grid = 3,
}

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
#[derive(Debug, Clone)]
#[repr(C)]
pub struct TreeSupportConfig {
    // ── General layer/resolution settings ──────────────────────────────

    /// Layer height in scaled coordinates.
    pub layer_height: CoordT,

    /// Resolution for polygon simplification in scaled coordinates.
    pub resolution: CoordT,

    /// Minimum feature size in scaled coordinates.
    pub min_feature_size: CoordT,

    // ── General support parameters ─────────────────────────────────────

    /// Maximum overhang angle (radians) before support is generated.
    pub support_angle: f64,

    /// Line width of support structures in scaled coordinates.
    pub support_line_width: CoordT,

    /// Line width of support roof in scaled coordinates.
    pub support_roof_line_width: CoordT,

    /// Whether support floor (bottom interface) is enabled.
    pub support_bottom_enable: bool,

    /// Height of support floor in scaled coordinates.
    pub support_bottom_height: CoordT,

    /// Whether support is limited to touching the build plate only.
    pub support_material_buildplate_only: bool,

    /// XY distance between support and model in scaled coordinates.
    pub support_xy_distance: CoordT,

    /// XY distance for the first layer in scaled coordinates.
    pub support_xy_distance_first_layer: CoordT,

    /// XY distance at overhang boundaries in scaled coordinates.
    pub support_xy_distance_overhang: CoordT,

    /// Top Z gap between model and support in scaled coordinates.
    pub support_top_distance: CoordT,

    /// Bottom Z gap between support and model/plate in scaled coordinates.
    pub support_bottom_distance: CoordT,

    /// Interface skip height in scaled coordinates.
    pub support_interface_skip_height: CoordT,

    /// Whether support roof is enabled.
    pub support_roof_enable: bool,

    /// Number of support roof layers.
    pub support_roof_layers: i32,

    /// Whether support floor is enabled.
    pub support_floor_enable: bool,

    /// Number of support floor layers.
    pub support_floor_layers: i32,

    /// Minimum roof area (scaled² units).
    pub minimum_roof_area: f64,

    /// Support roof interface pattern.
    pub roof_pattern: SupportInterfacePattern,

    /// Support body pattern.
    pub support_pattern: SupportPattern,

    /// Line spacing for support body in scaled coordinates.
    pub support_line_spacing: CoordT,

    /// Bottom offset for support in scaled coordinates.
    pub support_bottom_offset: CoordT,

    /// Number of support wall loops.
    pub support_wall_count: i32,

    /// Roof line distance in scaled coordinates.
    pub support_roof_line_distance: CoordT,

    /// Minimum support area in scaled coordinates.
    pub minimum_support_area: CoordT,

    /// Minimum bottom area in scaled coordinates.
    pub minimum_bottom_area: CoordT,

    /// Support offset in scaled coordinates.
    pub support_offset: CoordT,

    // ── Tree-specific parameters ───────────────────────────────────────

    /// Maximum branch angle when avoiding the model (radians).
    pub support_tree_angle: f64,

    /// Preferred branch angle without model avoidance (radians).
    pub support_tree_angle_slow: f64,

    /// Branch diameter at the base in scaled coordinates.
    pub support_tree_branch_diameter: CoordT,

    /// How much the branch thickens toward the base (radians).
    pub support_tree_branch_diameter_angle: f64,

    /// Distance between branch tips in scaled coordinates.
    pub support_tree_branch_distance: CoordT,

    /// Build plate contact diameter in scaled coordinates.
    pub support_tree_bp_diameter: CoordT,

    /// Tip density percentage (0–100).
    pub support_tree_top_rate: f64,

    /// Tip diameter in scaled coordinates.
    pub support_tree_tip_diameter: CoordT,

    /// Maximum diameter increase by merges when support rests on model, in
    /// scaled coordinates.
    pub support_tree_max_diameter_increase_by_merges_when_support_to_model: CoordT,

    /// Minimum height to model for tree support in scaled coordinates.
    pub support_tree_min_height_to_model: CoordT,

    // ── Interface preference ───────────────────────────────────────────

    /// How interface areas interact with support body.
    pub interface_preference: InterfacePreference,

    /// Whether support can rest on the model surface.
    pub support_rests_on_model: bool,
}

impl Default for TreeSupportConfig {
    /// Returns default configuration matching the C++ defaults.
    fn default() -> Self {
        Self {
            layer_height: scaled(0.15),
            resolution: scaled(0.025),
            min_feature_size: scaled(0.1),
            support_angle: 50.0 * PI / 180.0,
            support_line_width: scaled(0.4),
            support_roof_line_width: scaled(0.4),
            support_bottom_enable: false,
            support_bottom_height: scaled(1.0),
            support_material_buildplate_only: false,
            support_xy_distance: scaled(0.7),
            support_xy_distance_first_layer: scaled(0.7),
            support_xy_distance_overhang: scaled(0.2),
            support_top_distance: scaled(0.1),
            support_bottom_distance: scaled(0.1),
            support_interface_skip_height: scaled(0.3),
            support_roof_enable: false,
            support_roof_layers: 2,
            support_floor_enable: false,
            support_floor_layers: 2,
            minimum_roof_area: scaled(1_000_000.0) as f64, // scaled²
            roof_pattern: SupportInterfacePattern::Auto,
            support_pattern: SupportPattern::Rectilinear,
            support_line_spacing: scaled(2.66 - 0.4),
            support_bottom_offset: scaled(0.0),
            support_wall_count: 1,
            support_roof_line_distance: scaled(0.4),
            minimum_support_area: scaled(0.0),
            minimum_bottom_area: scaled(1.0),
            support_offset: scaled(0.0),
            support_tree_angle: 60.0 * PI / 180.0,
            support_tree_angle_slow: 50.0 * PI / 180.0,
            support_tree_branch_diameter: scaled(2.0),
            support_tree_branch_diameter_angle: 5.0 * PI / 180.0,
            support_tree_branch_distance: scaled(1.0),
            support_tree_bp_diameter: scaled(7.5),
            support_tree_top_rate: 15.0,
            support_tree_tip_diameter: scaled(0.4),
            support_tree_max_diameter_increase_by_merges_when_support_to_model: scaled(1.0),
            support_tree_min_height_to_model: scaled(1.0),
            interface_preference: InterfacePreference::Nothing,
            support_rests_on_model: false,
        }
    }
}

/// Derived/computed support settings.
///
/// These are computed from a [`TreeSupportConfig`] and cached for use during generation.
/// Mirrors the C++ `TreeSupportSettings` struct.
#[derive(Debug, Clone)]
pub struct DerivedSupportSettings {
    /// Effective branch radius (half of `support_tree_branch_diameter`).
    pub branch_radius: CoordT,

    /// Minimum branch radius (half of `support_tree_tip_diameter`).
    pub min_radius: CoordT,

    /// Maximum lateral movement per layer (from `support_tree_angle`).
    pub maximum_move_distance: CoordT,

    /// Maximum lateral movement per layer at slow angle.
    pub maximum_move_distance_slow: CoordT,

    /// Number of bottom/floor support layers.
    pub support_bottom_layers: usize,

    /// Number of layers at the tip before radius starts growing.
    pub tip_layers: usize,

    /// Radius increase per layer for the branch body.
    pub branch_radius_increase_per_layer: f64,

    /// Maximum radius increase when support rests on model.
    pub max_to_model_radius_increase: CoordT,

    /// Minimum distance-to-top when resting on model.
    pub min_dtt_to_model: usize,

    /// Radius at which increase rate changes.
    pub increase_radius_until_radius: CoordT,

    /// Layer at which radius increase rate changes.
    pub increase_radius_until_layer: usize,

    /// XY distance for collision checking.
    pub xy_distance: CoordT,

    /// Build plate radius.
    pub bp_radius: CoordT,

    /// Layer at which build plate radius increase begins.
    pub layer_start_bp_radius: i64,

    /// Build plate radius increase per layer.
    pub bp_radius_increase_per_layer: f64,

    /// Minimum XY distance (tighter than `xy_distance`).
    pub xy_min_distance: CoordT,

    /// Number of top gap layers.
    pub z_distance_top_layers: usize,

    /// Number of bottom gap layers.
    pub z_distance_bottom_layers: usize,
}

impl DerivedSupportSettings {
    /// Compute derived settings from a configuration.
    ///
    /// # Examples
    ///
    /// ```
    /// use orca_tree_supports::config::{TreeSupportConfig, DerivedSupportSettings};
    /// let cfg = TreeSupportConfig::default();
    /// let derived = DerivedSupportSettings::from_config(&cfg);
    /// assert!(derived.branch_radius > 0);
    /// ```
    pub fn from_config(config: &TreeSupportConfig) -> Self {
        let branch_radius = config.support_tree_branch_diameter / 2;
        let min_radius = config.support_tree_tip_diameter / 2;
        let layer_height_f = config.layer_height as f64;

        let maximum_move_distance =
            (config.support_tree_angle.tan() * layer_height_f) as CoordT;
        let maximum_move_distance_slow =
            (config.support_tree_angle_slow.tan() * layer_height_f) as CoordT;

        let branch_radius_increase_per_layer =
            config.support_tree_branch_diameter_angle.tan() * layer_height_f;

        // Number of tip layers: how many layers until the tip radius grows to
        // the full branch radius.
        let tip_layers = if branch_radius_increase_per_layer > 0.0 {
            ((branch_radius - min_radius) as f64 / branch_radius_increase_per_layer).ceil()
                as usize
        } else {
            0
        };

        let z_distance_top_layers = if config.layer_height > 0 {
            ((config.support_top_distance as f64) / layer_height_f).ceil() as usize
        } else {
            0
        };

        let z_distance_bottom_layers = if config.layer_height > 0 {
            ((config.support_bottom_distance as f64) / layer_height_f).ceil() as usize
        } else {
            0
        };

        let support_bottom_layers = if config.support_bottom_enable {
            if config.layer_height > 0 {
                ((config.support_bottom_height as f64) / layer_height_f).ceil() as usize
            } else {
                0
            }
        } else {
            0
        };

        let bp_radius = config.support_tree_bp_diameter / 2;
        let bp_radius_increase_per_layer = if bp_radius > branch_radius {
            // Grow toward the build plate radius over a fixed number of layers.
            let growth_layers = 10.0_f64; // heuristic matching C++
            (bp_radius - branch_radius) as f64 / growth_layers
        } else {
            0.0
        };

        let xy_min_distance = config.support_xy_distance_overhang;
        let increase_radius_until_radius = config.support_tree_branch_diameter;
        let increase_radius_until_layer = if branch_radius_increase_per_layer > 0.0 {
            (increase_radius_until_radius as f64 / branch_radius_increase_per_layer).ceil()
                as usize
        } else {
            0
        };

        let min_dtt_to_model = if config.layer_height > 0 {
            (config.support_tree_min_height_to_model as f64 / layer_height_f).ceil() as usize
        } else {
            0
        };

        Self {
            branch_radius,
            min_radius,
            maximum_move_distance,
            maximum_move_distance_slow,
            support_bottom_layers,
            tip_layers,
            branch_radius_increase_per_layer,
            max_to_model_radius_increase: config
                .support_tree_max_diameter_increase_by_merges_when_support_to_model,
            min_dtt_to_model,
            increase_radius_until_radius,
            increase_radius_until_layer,
            xy_distance: config.support_xy_distance,
            bp_radius,
            layer_start_bp_radius: 0,
            bp_radius_increase_per_layer,
            xy_min_distance,
            z_distance_top_layers,
            z_distance_bottom_layers,
        }
    }
}

impl TreeSupportConfig {
    /// Validate configuration parameters.
    ///
    /// Returns an error if any parameter is out of range.
    pub fn validate(&self) -> crate::utils::Result<()> {
        if self.layer_height <= 0 {
            return Err(crate::utils::TreeSupportError::InvalidConfig(
                "layer_height must be positive".into(),
            ));
        }
        if self.support_tree_angle <= 0.0 || self.support_tree_angle >= PI / 2.0 {
            return Err(crate::utils::TreeSupportError::InvalidConfig(
                "support_tree_angle must be in (0, π/2)".into(),
            ));
        }
        if self.support_tree_branch_diameter <= 0 {
            return Err(crate::utils::TreeSupportError::InvalidConfig(
                "support_tree_branch_diameter must be positive".into(),
            ));
        }
        if self.support_tree_top_rate < 0.0 || self.support_tree_top_rate > 100.0 {
            return Err(crate::utils::TreeSupportError::InvalidConfig(
                "support_tree_top_rate must be in [0, 100]".into(),
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config_is_valid() {
        let cfg = TreeSupportConfig::default();
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn test_derived_settings_branch_radius() {
        let cfg = TreeSupportConfig::default();
        let derived = DerivedSupportSettings::from_config(&cfg);
        assert_eq!(derived.branch_radius, cfg.support_tree_branch_diameter / 2);
        assert_eq!(derived.min_radius, cfg.support_tree_tip_diameter / 2);
    }

    #[test]
    fn test_derived_settings_movement() {
        let cfg = TreeSupportConfig::default();
        let derived = DerivedSupportSettings::from_config(&cfg);
        assert!(derived.maximum_move_distance > 0);
        assert!(derived.maximum_move_distance_slow > 0);
        assert!(derived.maximum_move_distance >= derived.maximum_move_distance_slow);
    }

    #[test]
    fn test_invalid_config_zero_layer_height() {
        let mut cfg = TreeSupportConfig::default();
        cfg.layer_height = 0;
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn test_invalid_config_bad_angle() {
        let mut cfg = TreeSupportConfig::default();
        cfg.support_tree_angle = PI; // 180° is invalid
        assert!(cfg.validate().is_err());
    }
}
