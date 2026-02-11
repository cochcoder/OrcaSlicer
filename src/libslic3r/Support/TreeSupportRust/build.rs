use std::env;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    // Only generate when building as a C library (not during tests/doc).
    if env::var("CARGO_CFG_TARGET_OS").is_ok() {
        let config = cbindgen::Config::from_file("cbindgen.toml")
            .unwrap_or_default();
        cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(config)
            .generate()
            .map(|bindings| {
                bindings.write_to_file("include/orca_tree_supports.h");
            })
            .ok(); // Don't fail the build if cbindgen can't generate
    }
}
