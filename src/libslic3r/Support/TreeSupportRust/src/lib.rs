//! # orca-tree-supports
//!
//! A Rust reimplementation of OrcaSlicer's tree/organic support generation subsystem.
//!
//! This crate provides the core algorithms for generating tree-style support structures
//! for FDM 3D printing, matching the behavior of the C++ `TreeSupport` implementation in
//! OrcaSlicer (a fork of Bambu Studio / PrusaSlicer).
//!
//! ## Architecture
//!
//! The crate is organized into the following modules:
//!
//! - [`config`]: Configuration parameters mirroring C++ `TreeSupportSettings`
//! - [`mesh`]: Indexed triangle mesh representation with AABB acceleration
//! - [`overhang_detection`]: Per-layer overhang area detection
//! - [`branch_generation`]: Tree branch placement, radius progression, and node dropping
//! - [`collision`]: Collision and avoidance volume computation using parry3d
//! - [`placement`]: Final geometry placement and polyline generation
//! - [`interface_layers`]: Support roof/floor and interface layer generation
//! - [`utils`]: Error types, profiling helpers, and common utilities
//! - [`ffi`]: C-compatible FFI boundary for integration with the C++ codebase
//!
//! ## FFI Usage
//!
//! The crate exposes an opaque-pointer API for C/C++ interop:
//!
//! ```c
//! #include "orca_tree_supports.h"
//!
//! OrcaTreeSupportConfig cfg = { /* ... */ };
//! OrcaMeshData mesh = { /* ... */ };
//! OrcaTreeSupportHandle *handle = orca_tree_support_create(&cfg, &mesh);
//! OrcaSupportOutput *output = orca_tree_support_generate(handle);
//! // ... use output ...
//! orca_tree_support_destroy_output(output);
//! orca_tree_support_destroy_handle(handle);
//! ```

pub mod branch_generation;
pub mod collision;
pub mod config;
pub mod ffi;
pub mod interface_layers;
pub mod mesh;
pub mod overhang_detection;
pub mod placement;
pub mod utils;
