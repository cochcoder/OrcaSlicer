// Test to trace the full pipeline output
use orca_tree_supports::config::{TreeSupportConfig, DerivedSupportSettings};
use orca_tree_supports::mesh::{MeshData, TriangleMesh};
use orca_tree_supports::overhang_detection::detect_overhangs;
use orca_tree_supports::branch_generation::{generate_contact_points, drop_nodes};
use orca_tree_supports::collision::CollisionModel;
use orca_tree_supports::interface_layers::generate_interface_layers;
use orca_tree_supports::placement::generate_support_output;
use orca_tree_supports::ffi::{orca_tree_support_create, orca_tree_support_generate, orca_tree_support_destroy_output, orca_tree_support_destroy_handle};

fn main() {
    // Create an overhang model: a box with a shelf that overhangs
    // Base: 20x20mm at z=0-10mm
    // Overhang shelf: extends 10mm outward at z=10-20mm
    // The model sits on the build plate (z_min=0)
    
    // For simplicity, create a shape with clear overhangs
    // A "T" shape: narrow base + wide top
    let vertices: Vec<f32> = vec![
        // Bottom base (narrow, 10x10mm)
        0.0, 0.0, 0.0,    // v0
        10.0, 0.0, 0.0,   // v1
        10.0, 10.0, 0.0,  // v2
        0.0, 10.0, 0.0,   // v3
        // Top of base (narrow, z=10)
        0.0, 0.0, 10.0,   // v4
        10.0, 0.0, 10.0,  // v5
        10.0, 10.0, 10.0, // v6
        0.0, 10.0, 10.0,  // v7
        // Wide top (extends 10mm each side, z=10)
        -10.0, -10.0, 10.0,  // v8
        20.0, -10.0, 10.0,   // v9
        20.0, 20.0, 10.0,    // v10
        -10.0, 20.0, 10.0,   // v11
        // Wide top z=20
        -10.0, -10.0, 20.0,  // v12
        20.0, -10.0, 20.0,   // v13
        20.0, 20.0, 20.0,    // v14
        -10.0, 20.0, 20.0,   // v15
    ];
    
    let indices: Vec<u32> = vec![
        // Bottom face of base
        0, 2, 1,  0, 3, 2,
        // Front face of base (z=0 to z=10)
        0, 1, 5,  0, 5, 4,
        // Back face of base
        2, 3, 7,  2, 7, 6,
        // Left face of base
        0, 4, 7,  0, 7, 3,
        // Right face of base
        1, 2, 6,  1, 6, 5,
        // Bottom face of wide top (the overhang!)
        8, 10, 9,  8, 11, 10,
        // Top face
        12, 13, 14,  12, 14, 15,
        // Front face of top
        8, 9, 13,  8, 13, 12,
        // Back face of top
        10, 11, 15,  10, 15, 14,
        // Left face of top
        8, 12, 15,  8, 15, 11,
        // Right face of top
        9, 10, 14,  9, 14, 13,
    ];
    
    let mesh_data = MeshData {
        vertices: vertices.as_ptr(),
        vertex_count: 16,
        indices: indices.as_ptr(),
        triangle_count: (indices.len() / 3) as u32,
    };
    
    let config = TreeSupportConfig::default();
    let settings = DerivedSupportSettings::from_config(&config);
    
    println!("=== Config ===");
    println!("layer_height (scaled): {}", config.layer_height);
    println!("layer_height (mm): {}", config.layer_height as f64 * 1e-6);
    println!("support_angle (rad): {:.4}", config.support_angle);
    println!("support_angle (deg): {:.1}", config.support_angle * 180.0 / std::f64::consts::PI);
    println!("tip_diameter (scaled): {}", config.support_tree_tip_diameter);
    println!("branch_diameter (scaled): {}", config.support_tree_branch_diameter);
    println!("branch_distance (scaled): {}", config.support_tree_branch_distance);
    println!("min_radius: {}", settings.min_radius);
    println!("branch_radius: {}", settings.branch_radius);
    println!("max_move_distance: {}", settings.maximum_move_distance);
    println!("branch_radius_increase_per_layer: {:.2}", settings.branch_radius_increase_per_layer);
    
    // Test via FFI
    println!("\n=== FFI Pipeline ===");
    unsafe {
        let handle = orca_tree_support_create(&config, &mesh_data);
        if handle.is_null() {
            println!("ERROR: handle is null!");
            return;
        }
        
        let output = orca_tree_support_generate(handle);
        if output.is_null() {
            println!("ERROR: output is null!");
            orca_tree_support_destroy_handle(handle);
            return;
        }
        
        let out = &*output;
        println!("success: {}", out.success);
        println!("layer_count: {}", out.layer_count);
        println!("branch_count: {}", out.branch_count);
        
        if out.layer_count > 0 && !out.layers.is_null() {
            let layers = std::slice::from_raw_parts(out.layers, out.layer_count as usize);
            for (i, layer) in layers.iter().enumerate() {
                if i < 5 || i >= layers.len() - 3 {
                    println!("  layer[{}]: z={:.3}mm, polygons={}, total_points={}",
                        i, layer.z, layer.polygon_count, layer.total_point_count);
                } else if i == 5 {
                    println!("  ... ({} layers total) ...", layers.len());
                }
            }
        }
        
        orca_tree_support_destroy_output(output);
        orca_tree_support_destroy_handle(handle);
    }
    
    // Also test the internal pipeline directly
    println!("\n=== Internal Pipeline ===");
    let mesh = unsafe { TriangleMesh::from_ffi(&mesh_data).unwrap() };
    let layer_height_mm = config.layer_height as f64 * 1e-6;
    let layer_heights = mesh.layer_z_values(layer_height_mm);
    println!("mesh z_range: {:?}", mesh.z_range());
    println!("layer_height_mm: {}", layer_height_mm);
    println!("num layers: {}", layer_heights.len());
    if !layer_heights.is_empty() {
        println!("first layer z: {:.4}", layer_heights[0]);
        println!("last layer z: {:.4}", layer_heights[layer_heights.len()-1]);
    }
    
    let overhangs = detect_overhangs(&mesh, &config, &layer_heights);
    println!("overhang total_regions: {}", overhangs.total_regions);
    let mut layers_with_overhangs = 0;
    for (i, layer) in overhangs.layers.iter().enumerate() {
        if !layer.is_empty() {
            layers_with_overhangs += 1;
            if layers_with_overhangs <= 3 {
                println!("  overhang layer[{}] (z={:.3}): {} regions", i, layer_heights[i], layer.len());
                for (j, area) in layer.iter().enumerate().take(2) {
                    println!("    region[{}]: boundary_pts={}, area={:.0}", j, area.boundary.len(), area.area);
                }
            }
        }
    }
    println!("layers with overhangs: {}", layers_with_overhangs);
    
    let contact_nodes = generate_contact_points(&overhangs, &config, &settings, &layer_heights);
    println!("contact_nodes: {}", contact_nodes.len());
    for (i, node) in contact_nodes.iter().enumerate().take(3) {
        println!("  node[{}]: pos=({}, {}), z={:.3}, layer={}, radius={}", 
            i, node.position.x, node.position.y, node.z, node.layer_index, node.radius);
    }
    
    let collision_model = CollisionModel::new(mesh.clone(), settings.clone(), layer_heights.clone());
    let branches = drop_nodes(&contact_nodes, &config, &settings, &collision_model, &layer_heights);
    println!("branches: {}", branches.branches.len());
    println!("total_nodes: {}", branches.total_nodes);
    for (i, branch) in branches.branches.iter().enumerate().take(3) {
        println!("  branch[{}]: {} nodes, reaches_bp={}", i, branch.nodes.len(), branch.reaches_buildplate);
        if let Some(first) = branch.nodes.first() {
            println!("    top: layer={}, radius={}", first.layer_index, first.radius);
        }
        if let Some(last) = branch.nodes.last() {
            println!("    bottom: layer={}, radius={}", last.layer_index, last.radius);
        }
    }
    
    let interface = generate_interface_layers(&branches.branches, &config, &layer_heights);
    let output = generate_support_output(&branches, &interface, &config, &layer_heights);
    println!("output layers: {}", output.layers.len());
    println!("output branches: {}", output.branch_count);
    for (i, layer) in output.layers.iter().enumerate().take(5) {
        let total_pts: usize = layer.polygons.iter().map(|p| p.len()).sum();
        println!("  output_layer[{}]: z={:.3}, polygons={}, total_pts={}", 
            i, layer.z, layer.polygons.len(), total_pts);
    }
}
