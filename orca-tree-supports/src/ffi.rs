//! C-compatible FFI boundary for tree support generation.
//!
//! This module provides the `extern "C"` API that allows the C++ OrcaSlicer
//! codebase to call into the Rust tree support generator. It uses opaque pointer
//! handles for all complex types and plain C structs for configuration.
//!
//! # Usage from C/C++
//!
//! ```c
//! #include "orca_tree_supports.h"
//!
//! OrcaTreeSupportConfig config = { /* fill in parameters */ };
//! OrcaMeshData mesh = { vertices_ptr, vertex_count, indices_ptr, triangle_count };
//!
//! OrcaTreeSupportHandle *handle = orca_tree_support_create(&config, &mesh);
//! if (handle == NULL) { /* error */ }
//!
//! OrcaSupportOutput *output = orca_tree_support_generate(handle);
//! if (output == NULL || !output->success) { /* error */ }
//!
//! // Use output->layers[i] for each layer's polygons
//!
//! orca_tree_support_destroy_output(output);
//! orca_tree_support_destroy_handle(handle);
//! ```
//!
//! # Safety
//!
//! All FFI functions document their safety requirements. The caller must:
//! - Pass valid, non-null pointers where required.
//! - Not use a handle or output after it has been destroyed.
//! - Not free the mesh data while a handle referencing it is alive.

use crate::branch_generation::{drop_nodes, generate_contact_points};
use crate::collision::CollisionModel;
use crate::config::{DerivedSupportSettings, TreeSupportConfig};
use crate::interface_layers::generate_interface_layers;
use crate::mesh::{MeshData, TriangleMesh};
use crate::overhang_detection::detect_overhangs;
use crate::placement::{
    generate_support_output, SupportLayerOutput, SupportOutput, SupportOutputOwned,
};
use crate::utils::Point2D;

/// Opaque handle to a tree support generation session.
///
/// Created by [`orca_tree_support_create`] and destroyed by
/// [`orca_tree_support_destroy_handle`]. Do not access the internal fields
/// directly from C/C++ code.
pub struct TreeSupportHandle {
    config: TreeSupportConfig,
    settings: DerivedSupportSettings,
    mesh: TriangleMesh,
    layer_heights: Vec<f64>,
}

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
#[no_mangle]
pub unsafe extern "C" fn orca_tree_support_create(
    config: *const TreeSupportConfig,
    mesh: *const MeshData,
) -> *mut TreeSupportHandle {
    if config.is_null() || mesh.is_null() {
        return std::ptr::null_mut();
    }

    let config = match std::ptr::read(config).validate() {
        Ok(()) => std::ptr::read(config),
        Err(_) => {
            // Re-read since validate consumed it conceptually.
            return std::ptr::null_mut();
        }
    };

    let mesh_ref = &*mesh;
    let triangle_mesh = match TriangleMesh::from_ffi(mesh_ref) {
        Ok(m) => m,
        Err(_) => return std::ptr::null_mut(),
    };

    let settings = DerivedSupportSettings::from_config(&config);
    let layer_height_mm = config.layer_height as f64 * 1e-6; // scaled → mm
    let layer_heights = triangle_mesh.layer_z_values(layer_height_mm);

    let handle = Box::new(TreeSupportHandle {
        config,
        settings,
        mesh: triangle_mesh,
        layer_heights,
    });

    Box::into_raw(handle)
}

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
#[no_mangle]
pub unsafe extern "C" fn orca_tree_support_generate(
    handle: *mut TreeSupportHandle,
) -> *mut SupportOutput {
    if handle.is_null() {
        return std::ptr::null_mut();
    }

    let handle = &*handle;

    // Step 1: Detect overhangs.
    let overhangs = detect_overhangs(&handle.mesh, &handle.config, &handle.layer_heights);

    // Step 2: Build collision model.
    let collision_model = CollisionModel::new(
        handle.mesh.clone(),
        handle.settings.clone(),
        handle.layer_heights.clone(),
    );

    // Step 3: Generate contact points.
    let contact_nodes = generate_contact_points(
        &overhangs,
        &handle.config,
        &handle.settings,
        &handle.layer_heights,
    );

    // Step 4: Drop nodes (grow branches downward).
    let branches = drop_nodes(
        &contact_nodes,
        &handle.config,
        &handle.settings,
        &collision_model,
        &handle.layer_heights,
    );

    // Step 5: Generate interface layers.
    let interface = generate_interface_layers(
        &branches.branches,
        &handle.config,
        &handle.layer_heights,
    );

    // Step 6: Generate final output.
    let output_owned = generate_support_output(
        &branches,
        &interface,
        &handle.config,
        &handle.layer_heights,
    );

    // Convert to FFI output.
    let ffi_output = convert_to_ffi_output(output_owned);
    Box::into_raw(Box::new(ffi_output))
}

/// Destroy a tree support handle and free its resources.
///
/// # Safety
///
/// - `handle` must be a valid pointer returned by [`orca_tree_support_create`],
///   or null (in which case this function is a no-op).
/// - The handle must not be used after this call.
#[no_mangle]
pub unsafe extern "C" fn orca_tree_support_destroy_handle(handle: *mut TreeSupportHandle) {
    if !handle.is_null() {
        drop(Box::from_raw(handle));
    }
}

/// Destroy a support output and free its resources.
///
/// # Safety
///
/// - `output` must be a valid pointer returned by [`orca_tree_support_generate`],
///   or null (in which case this function is a no-op).
/// - The output must not be used after this call.
#[no_mangle]
pub unsafe extern "C" fn orca_tree_support_destroy_output(output: *mut SupportOutput) {
    if !output.is_null() {
        let output = Box::from_raw(output);
        if !output.layers.is_null() && output.layer_count > 0 {
            // Reconstruct the Vec to properly free the layer data.
            let layers = Vec::from_raw_parts(
                output.layers as *mut SupportLayerOutput,
                output.layer_count as usize,
                output.layer_count as usize,
            );
            for layer in &layers {
                if !layer.polygon_points.is_null() && layer.total_point_count > 0 {
                    let _points = Vec::from_raw_parts(
                        layer.polygon_points as *mut Point2D,
                        layer.total_point_count as usize,
                        layer.total_point_count as usize,
                    );
                }
                if !layer.polygon_sizes.is_null() && layer.polygon_count > 0 {
                    let _sizes = Vec::from_raw_parts(
                        layer.polygon_sizes as *mut u32,
                        layer.polygon_count as usize,
                        layer.polygon_count as usize,
                    );
                }
            }
        }
    }
}

/// Convert the owned Rust output to the FFI-compatible C struct.
fn convert_to_ffi_output(owned: SupportOutputOwned) -> SupportOutput {
    let mut ffi_layers: Vec<SupportLayerOutput> = Vec::with_capacity(owned.layers.len());

    for layer in &owned.layers {
        let mut all_points: Vec<Point2D> = Vec::new();
        let mut polygon_sizes: Vec<u32> = Vec::new();

        for polygon in &layer.polygons {
            polygon_sizes.push(polygon.len() as u32);
            all_points.extend_from_slice(polygon);
        }

        let total_point_count = all_points.len() as u32;
        let polygon_count = polygon_sizes.len() as u32;

        let points_ptr = if all_points.is_empty() {
            std::ptr::null()
        } else {
            let ptr = all_points.as_ptr();
            std::mem::forget(all_points);
            ptr
        };

        let sizes_ptr = if polygon_sizes.is_empty() {
            std::ptr::null()
        } else {
            let ptr = polygon_sizes.as_ptr();
            std::mem::forget(polygon_sizes);
            ptr
        };

        ffi_layers.push(SupportLayerOutput {
            z: layer.z,
            layer_index: layer.layer_index as u32,
            polygon_count,
            polygon_points: points_ptr,
            total_point_count,
            polygon_sizes: sizes_ptr,
        });
    }

    let layer_count = ffi_layers.len() as u32;
    let layers_ptr = if ffi_layers.is_empty() {
        std::ptr::null()
    } else {
        let ptr = ffi_layers.as_ptr();
        std::mem::forget(ffi_layers);
        ptr
    };

    SupportOutput {
        layer_count,
        layers: layers_ptr,
        branch_count: owned.branch_count as u32,
        success: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_null_config() {
        unsafe {
            let handle = orca_tree_support_create(std::ptr::null(), std::ptr::null());
            assert!(handle.is_null());
        }
    }

    #[test]
    fn test_destroy_null_handle() {
        unsafe {
            orca_tree_support_destroy_handle(std::ptr::null_mut());
        }
    }

    #[test]
    fn test_destroy_null_output() {
        unsafe {
            orca_tree_support_destroy_output(std::ptr::null_mut());
        }
    }

    #[test]
    fn test_generate_null_handle() {
        unsafe {
            let output = orca_tree_support_generate(std::ptr::null_mut());
            assert!(output.is_null());
        }
    }

    #[test]
    fn test_full_ffi_roundtrip() {
        unsafe {
            // Create a simple mesh.
            let vertices: Vec<f32> = vec![
                0.0, 0.0, 0.0, // v0
                10.0, 0.0, 0.0, // v1
                5.0, 10.0, 0.0, // v2
                5.0, 5.0, 10.0, // v3
            ];
            let indices: Vec<u32> = vec![0, 1, 2, 0, 1, 3, 1, 2, 3, 0, 2, 3];

            let mesh_data = MeshData {
                vertices: vertices.as_ptr(),
                vertex_count: 4,
                indices: indices.as_ptr(),
                triangle_count: 4,
            };

            let config = TreeSupportConfig::default();

            let handle = orca_tree_support_create(&config, &mesh_data);
            assert!(!handle.is_null());

            let output = orca_tree_support_generate(handle);
            assert!(!output.is_null());
            assert!((*output).success);

            orca_tree_support_destroy_output(output);
            orca_tree_support_destroy_handle(handle);
        }
    }
}
