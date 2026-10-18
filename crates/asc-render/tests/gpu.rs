use asc_kn5 as kn5;
use asc_render as render;
use asc_render::Resources;
use asc_test_support as support;
use asc_texture as dds;
use glam::{Mat4, Vec3};
use std::io::Cursor;

#[test]
#[ignore = "Requires a native GPU adapter or software Vulkan; run explicitly in GPU CI."]
fn native_gpu_draws_geometry_obeys_depth_and_unpads_readback() {
    let points = [[-2., 0., -2.], [2., 0., -2.], [2., 0., 2.], [-2., 0., 2.]];
    let mesh = support::mesh(&points, &[0, 1, 2, 0, 2, 3], 2, 1, 1, 0., 0);
    let upper = support::base(
        std::slice::from_ref(&mesh),
        Mat4::from_translation(Vec3::new(0., 0.2, 0.)).to_cols_array(),
        1,
    );
    let mut scene = kn5::parse(Cursor::new(support::file(
        support::base(&[mesh, upper], Mat4::IDENTITY.to_cols_array(), 1),
        6,
    )))
    .unwrap_or_else(|e| panic!("{e}"));
    // Submit near green layer first and far red layer last. Without depth testing,
    // the center would be red. Colours are original test inputs, not materials.
    for vertex in &mut scene.vertices[..4] {
        vertex.colour = [0.9, 0.01, 0.01];
    }
    for vertex in &mut scene.vertices[4..] {
        vertex.colour = [0.01, 0.9, 0.01];
    }
    scene.indices.rotate_left(6);
    let fallback = std::env::var_os("OPENASC_TEST_FALLBACK").is_some();
    let frame = pollster::block_on(render::render(&scene, 319, 177, fallback)).unwrap();
    assert_eq!(frame.rgba.len(), 319 * 177 * 4);
    assert!(frame.report.non_background_pixels > 1000);
    assert!(frame.report.non_background_pixels < 319 * 177 - 1000);
    assert!(
        frame.rgba[..4]
            .iter()
            .zip(frame.report.background_rgba)
            .all(|(&v, b)| v.abs_diff(b) <= 1)
    );
    let pixel = &frame.rgba[(88 * 319 + 159) * 4..][..4];
    assert!(
        pixel[1] > pixel[0] * 2 && pixel[1] > pixel[2] * 2,
        "Depth test center: {pixel:?}"
    );
    assert_eq!(pixel[3], 255);
    let path = support::temp_path("native-gpu").with_extension("png");
    render::save_png(&frame, &path).unwrap();
    assert!(
        render::save_png(&frame, &path).is_err(),
        "Must not overwrite captures"
    );
    let decoder = png::Decoder::new(std::io::BufReader::new(std::fs::File::open(&path).unwrap()));
    let mut reader = decoder.read_info().unwrap();
    let mut pixels = vec![0; reader.output_buffer_size().unwrap()];
    let info = reader.next_frame(&mut pixels).unwrap();
    assert_eq!((info.width, info.height), (319, 177));
    assert_eq!(&pixels[..info.buffer_size()], &frame.rgba);
    std::fs::remove_file(&path).unwrap();
    println!("{}", serde_json::to_string(&frame.report).unwrap());
}

fn plane_scene(heights: &[f32]) -> kn5::Scene {
    let points = [[-2., 0., -2.], [2., 0., -2.], [2., 0., 2.], [-2., 0., 2.]];
    let children: Vec<_> = heights
        .iter()
        .map(|&y| {
            support::base(
                &[support::mesh(&points, &[0, 1, 2, 0, 2, 3], 2, 1, 1, 0., 0)],
                Mat4::from_translation(Vec3::new(0., y, 0.)).to_cols_array(),
                1,
            )
        })
        .collect();
    let mut scene = kn5::parse(Cursor::new(support::file(
        support::base(&children, Mat4::IDENTITY.to_cols_array(), 1),
        6,
    )))
    .unwrap_or_else(|e| panic!("{e}"));
    for vertex in &mut scene.vertices {
        vertex.uv = [
            (vertex.position[0] + 2.) / 4.,
            (vertex.position[2] + 2.) / 4.,
        ];
    }
    scene.materials[0].diffuse = 1.;
    scene
}
fn texture(width: u32, height: u32, pixels: Vec<u8>) -> dds::Texture {
    dds::Texture {
        encoded: None,
        info: dds::Info {
            width,
            height,
            format: "RGB",
            source_mips: 1,
            decoded_bytes: pixels.len(),
            generated_mips: false,
        },
        levels: vec![dds::Level {
            width,
            height,
            rgba: pixels,
        }],
    }
}
fn resources(colour: [u8; 4]) -> Resources {
    Resources {
        allow_bc: false,
        textures: vec![texture(1, 1, colour.to_vec())],
        material_textures: vec![Some(0)],
        diagnostics: Vec::new(),
    }
}
fn frame(scene: &kn5::Scene, resources: &Resources) -> render::Frame {
    pollster::block_on(render::render_with_textures(
        scene,
        Some(resources),
        319,
        177,
        std::env::var_os("OPENASC_TEST_FALLBACK").is_some(),
    ))
    .unwrap()
}
fn pixel<'a>(frame: &'a render::Frame, scene: &kn5::Scene, point: Vec3) -> &'a [u8] {
    let camera =
        Mat4::from_cols_array(&render::camera(scene.summary.bounds.unwrap(), 319, 177).unwrap());
    let p = camera * point.extend(1.);
    let x = ((p.x * 0.5 + 0.5) * 319.).floor() as usize;
    let y = ((0.5 - p.y * 0.5) * 177.).floor() as usize;
    &frame.rgba[(y * 319 + x) * 4..][..4]
}

#[test]
#[ignore = "Requires native GPU or software Vulkan; run explicitly in GPU CI."]
fn native_gpu_samples_uv_without_vertical_flip() {
    let scene = plane_scene(&[0.]);
    let r = Resources {
        allow_bc: false,
        textures: vec![texture(
            2,
            2,
            vec![
                255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 0, 255,
            ],
        )],
        material_textures: vec![Some(0)],
        diagnostics: Vec::new(),
    };
    let image = frame(&scene, &r);
    for (p, expected) in [
        (Vec3::new(-1., 0., -1.), [true, false, false]),
        (Vec3::new(1., 0., -1.), [false, true, false]),
        (Vec3::new(-1., 0., 1.), [false, false, true]),
        (Vec3::new(1., 0., 1.), [true, true, false]),
    ] {
        let colour = pixel(&image, &scene, p);
        for (actual, bright) in colour.iter().take(3).zip(expected) {
            assert!(
                if bright { *actual > 200 } else { *actual < 65 },
                "UV pixel {p:?}: {colour:?}"
            );
        }
    }
    assert_eq!(
        (image.report.uploaded_textures, image.report.textured_draws),
        (1, 1)
    );
}

#[test]
#[ignore = "Requires native GPU or software Vulkan; run explicitly in GPU CI."]
fn native_gpu_opaque_cutout_opacity_and_linear_alpha_blend_are_distinct() {
    let mut scene = plane_scene(&[0.]);
    let mut r = resources([240, 30, 10, 0]);
    let opaque = frame(&scene, &r);
    let solid = pixel(&opaque, &scene, Vec3::ZERO);
    assert!(
        solid[0] > 200 && solid[1] < 40,
        "Opaque must ignore texture alpha: {solid:?}"
    );
    scene.materials[0].alpha_test = true;
    let cutout = frame(&scene, &r);
    assert_eq!(cutout.report.non_background_pixels, 0);
    scene.materials[0].alpha_test = false;
    scene.materials[0].blend = 2;
    assert_eq!(frame(&scene, &r).report.non_background_pixels, 0);
    scene.materials[0].blend = 0;
    scene.materials[0].alpha_test = true;
    scene.materials[0].opacity = 0.4;
    r.textures[0].levels[0].rgba[3] = 255;
    assert_eq!(frame(&scene, &r).report.non_background_pixels, 0);
    scene.materials[0].alpha_test = false;
    scene.materials[0].blend = 1;
    scene.materials[0].opacity = 1.;
    r.textures[0].levels[0].rgba[3] = 128;
    let blend = frame(&scene, &r);
    let p = pixel(&blend, &scene, Vec3::ZERO);
    let linear = |byte: u8| {
        let v = byte as f64 / 255.;
        if v <= 0.04045 {
            v / 12.92
        } else {
            ((v + 0.055) / 1.055).powf(2.4)
        }
    };
    let encode = |v: f64| {
        let c = if v <= 0.0031308 {
            v * 12.92
        } else {
            1.055 * v.powf(1. / 2.4) - 0.055
        };
        (c * 255.).round() as u8
    };
    for c in 0..3 {
        let a = 128. / 255.;
        let expected =
            encode(linear(solid[c]) * a + linear(blend.report.background_rgba[c]) * (1. - a));
        assert!(
            p[c].abs_diff(expected) <= 3,
            "Blend channel {c}: {p:?}, expected {expected}"
        );
    }
    assert_eq!(p[3], 255);
    assert_eq!(blend.report.blended_draws, 1);
}

#[test]
#[ignore = "Requires native GPU or software Vulkan; run explicitly in GPU CI."]
fn native_gpu_read_only_and_disabled_depth_states_affect_occlusion() {
    let mut scene = plane_scene(&[0., 0.2, 0.1]);
    for _ in 0..2 {
        scene.materials.push(scene.materials[0].clone());
    }
    for (i, draw) in scene.draws.iter_mut().enumerate() {
        draw.material = i;
    }
    for (i, vertices) in scene.vertices.chunks_mut(4).enumerate() {
        for v in vertices {
            v.colour = match i {
                0 => [0., 0.9, 0.],
                1 => [0.9, 0., 0.],
                _ => [0., 0., 0.9],
            };
        }
    }
    scene.materials[1].depth = 1;
    scene.materials[2].depth = 1;
    let image = pollster::block_on(render::render(
        &scene,
        319,
        177,
        std::env::var_os("OPENASC_TEST_FALLBACK").is_some(),
    ))
    .unwrap();
    let p = pixel(&image, &scene, Vec3::new(0., 0.1, 0.));
    assert!(
        p[2] > 200 && p[0] < 20,
        "Far read-only overlay should pass after near read-only overlay: {p:?}"
    );
    scene.materials[2].depth = 2;
    for v in &mut scene.vertices[8..] {
        v.position[1] = -0.2;
    }
    let image = pollster::block_on(render::render(
        &scene,
        319,
        177,
        std::env::var_os("OPENASC_TEST_FALLBACK").is_some(),
    ))
    .unwrap();
    let p = pixel(&image, &scene, Vec3::ZERO);
    assert!(
        p[2] > 200 && p[1] < 20,
        "Disabled depth must draw even behind the base: {p:?}"
    );
}

#[test]
#[ignore = "Requires native GPU or software Vulkan; run explicitly in GPU CI."]
fn native_gpu_sorts_transparency_by_camera_depth_independently_of_source_order() {
    let mut scene = plane_scene(&[0.2, 0.]);
    scene.materials.push(scene.materials[0].clone());
    scene.draws[1].material = 1;
    for m in &mut scene.materials {
        m.blend = 1;
    }
    let r = Resources {
        allow_bc: false,
        textures: vec![
            texture(1, 1, vec![0, 255, 0, 128]),
            texture(1, 1, vec![255, 0, 0, 128]),
        ],
        material_textures: vec![Some(0), Some(1)],
        diagnostics: Vec::new(),
    };
    let first = frame(&scene, &r);
    scene.draws.reverse();
    let second = frame(&scene, &r);
    assert_eq!(first.rgba, second.rgba);
    let p = pixel(&first, &scene, Vec3::new(0., 0.1, 0.));
    assert!(
        p[1] > p[0] + 30,
        "Near green blend should contribute more than far red: {p:?}"
    );
}

#[test]
#[ignore = "Requires native GPU or software Vulkan; run explicitly in GPU CI."]
fn bc_upload_and_no_bc_device_fallback_preserve_colour_alpha_and_mip_tails() {
    for fourcc in [*b"DXT1", *b"DXT3", *b"DXT5"] {
        let mut data = support::bc1(0xf800);
        data[12..16].copy_from_slice(&4u32.to_le_bytes());
        data[16..20].copy_from_slice(&12u32.to_le_bytes());
        data[28..32].copy_from_slice(&4u32.to_le_bytes());
        data[84..88].copy_from_slice(&fourcc);
        data.truncate(128);
        for (count, colour) in [(3, 0xf800), (2, 0x07e0), (1, 0x001f), (1, 0xffff)] {
            let block = support::bc1(colour);
            for _ in 0..count {
                if fourcc == *b"DXT3" {
                    data.extend([0x88; 8]);
                }
                if fourcc == *b"DXT5" {
                    data.extend([128, 128, 0, 0, 0, 0, 0, 0]);
                }
                data.extend(&block[128..]);
            }
        }
        let mut scene = plane_scene(&[0.]);
        scene.materials[0].blend = 1;
        for minified in [false, true] {
            if minified {
                for vertex in &mut scene.vertices {
                    vertex.uv = vertex.uv.map(|v| v * 256.);
                }
            }
            let mut r = Resources {
                textures: vec![dds::prepare(&data, 1024).unwrap()],
                material_textures: vec![Some(0)],
                diagnostics: Vec::new(),
                allow_bc: true,
            };
            let direct = frame(&scene, &r);
            assert!(
                direct.report.diagnostics.is_empty(),
                "{:?}",
                direct.report.diagnostics
            );
            assert_eq!(
                direct.report.compressed_textures,
                usize::from(direct.report.bc_enabled)
            );
            r.allow_bc = false;
            let decoded = frame(&scene, &r);
            assert!(!decoded.report.bc_enabled);
            assert_eq!(decoded.report.compressed_textures, 0);
            assert!(
                decoded.report.diagnostics.is_empty(),
                "{:?}",
                decoded.report.diagnostics
            );
            let a = pixel(&direct, &scene, Vec3::ZERO);
            let b = pixel(&decoded, &scene, Vec3::ZERO);
            assert!(
                a.iter().zip(b).all(|(x, y)| x.abs_diff(*y) <= 2),
                "BC/RGBA centre {fourcc:?}: {a:?}/{b:?}"
            );
            assert!(a[0] > 80, "Red base/white mip must contribute: {a:?}");
            if minified {
                assert!(
                    a[1] > 80 && a[2] > 80,
                    "Authored white 1x1 mip must be sampled: {a:?}"
                );
            }
            if direct.report.bc_enabled {
                assert!(direct.report.texture_upload_bytes < decoded.report.texture_upload_bytes);
            }
        }
    }
}
