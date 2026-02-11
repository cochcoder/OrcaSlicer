# CMakeLists.txt for the Rust tree support library (orca-tree-supports)
#
# This file integrates the Rust crate into the CMake build system.
# It invokes `cargo build` to compile the Rust static library and
# creates an IMPORTED target that can be linked by libslic3r.
#
# Requirements:
#   - Rust toolchain (rustc, cargo) must be installed and on PATH
#   - Minimum Rust version: 1.70 (2021 edition)

# Detect cargo
find_program(CARGO_EXECUTABLE cargo HINTS $ENV{HOME}/.cargo/bin)
if(NOT CARGO_EXECUTABLE)
    message(WARNING "cargo not found — Rust tree support library will NOT be built. "
                    "Install Rust from https://rustup.rs/ to enable.")
    return()
endif()

message(STATUS "Found cargo: ${CARGO_EXECUTABLE}")

# Determine build type for cargo
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CARGO_BUILD_TYPE "debug")
    set(CARGO_BUILD_FLAGS "")
else()
    set(CARGO_BUILD_TYPE "release")
    set(CARGO_BUILD_FLAGS "--release")
endif()

# Paths
set(RUST_CRATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(RUST_TARGET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/target")
set(RUST_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include/orca_tree_supports.h")

# Determine the static library filename (platform-dependent)
if(WIN32)
    set(RUST_LIB_NAME "orca_tree_supports.lib")
elseif(APPLE)
    set(RUST_LIB_NAME "liborca_tree_supports.a")
else()
    set(RUST_LIB_NAME "liborca_tree_supports.a")
endif()

set(RUST_LIB_PATH "${RUST_TARGET_DIR}/${CARGO_BUILD_TYPE}/${RUST_LIB_NAME}")

# Custom command: build the Rust crate
add_custom_command(
    OUTPUT "${RUST_LIB_PATH}"
    COMMAND ${CARGO_EXECUTABLE} build ${CARGO_BUILD_FLAGS}
            --manifest-path "${RUST_CRATE_DIR}/Cargo.toml"
            --target-dir "${RUST_TARGET_DIR}"
    WORKING_DIRECTORY "${RUST_CRATE_DIR}"
    DEPENDS
        "${RUST_CRATE_DIR}/Cargo.toml"
        "${RUST_CRATE_DIR}/Cargo.lock"
        "${RUST_CRATE_DIR}/src/lib.rs"
        "${RUST_CRATE_DIR}/src/config.rs"
        "${RUST_CRATE_DIR}/src/mesh.rs"
        "${RUST_CRATE_DIR}/src/overhang_detection.rs"
        "${RUST_CRATE_DIR}/src/branch_generation.rs"
        "${RUST_CRATE_DIR}/src/collision.rs"
        "${RUST_CRATE_DIR}/src/interface_layers.rs"
        "${RUST_CRATE_DIR}/src/placement.rs"
        "${RUST_CRATE_DIR}/src/ffi.rs"
        "${RUST_CRATE_DIR}/src/utils.rs"
    COMMENT "Building Rust tree support library (${CARGO_BUILD_TYPE})..."
)

# Custom target that triggers the build
add_custom_target(orca_tree_supports_build
    DEPENDS "${RUST_LIB_PATH}"
)

# Create an IMPORTED static library target
add_library(orca_tree_supports STATIC IMPORTED GLOBAL)
set_target_properties(orca_tree_supports PROPERTIES
    IMPORTED_LOCATION "${RUST_LIB_PATH}"
    INTERFACE_INCLUDE_DIRECTORIES "${RUST_CRATE_DIR}/include"
)
add_dependencies(orca_tree_supports orca_tree_supports_build)

# On Unix, Rust static libs need to link against system threading and dl libs
if(UNIX AND NOT APPLE)
    set_target_properties(orca_tree_supports PROPERTIES
        INTERFACE_LINK_LIBRARIES "pthread;dl;m"
    )
elseif(APPLE)
    set_target_properties(orca_tree_supports PROPERTIES
        INTERFACE_LINK_LIBRARIES "-framework Security;-framework CoreFoundation"
    )
elseif(WIN32)
    set_target_properties(orca_tree_supports PROPERTIES
        INTERFACE_LINK_LIBRARIES "ws2_32;userenv;bcrypt;ntdll"
    )
endif()

# Custom target to run Rust tests
add_custom_target(orca_tree_supports_test
    COMMAND ${CARGO_EXECUTABLE} test
            --manifest-path "${RUST_CRATE_DIR}/Cargo.toml"
            --target-dir "${RUST_TARGET_DIR}"
    WORKING_DIRECTORY "${RUST_CRATE_DIR}"
    COMMENT "Running Rust tree support tests..."
)
