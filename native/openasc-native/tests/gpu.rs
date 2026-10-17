mod support;
use glam::{Mat4, Vec3};
use openasc_native::{kn5, render};
use std::{
    io::Cursor,
    time::{SystemTime, UNIX_EPOCH},
};

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
    let serial = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let path = std::env::temp_dir().join(format!(
        "openasc-native-gpu-{}-{serial}.png",
        std::process::id()
    ));
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
