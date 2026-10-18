use asc_texture as dds;

struct Header {
    width: u32,
    height: u32,
    fourcc: [u8; 4],
    bits: u32,
    masks: [u32; 4],
    pf: u32,
    flags: u32,
    pitch: u32,
    mips: u32,
    caps: u32,
}
impl Default for Header {
    fn default() -> Self {
        Self {
            width: 4,
            height: 4,
            fourcc: *b"DXT1",
            bits: 0,
            masks: [0; 4],
            pf: 4,
            flags: 0x1007,
            pitch: 0,
            mips: 1,
            caps: 0,
        }
    }
}
fn source(h: Header, pixels: &[u8]) -> Vec<u8> {
    let mut header = [0u32; 31];
    header[..7].copy_from_slice(&[124, h.flags, h.height, h.width, h.pitch, 0, h.mips]);
    header[18..26].copy_from_slice(&[
        32,
        h.pf,
        u32::from_le_bytes(h.fourcc),
        h.bits,
        h.masks[0],
        h.masks[1],
        h.masks[2],
        h.masks[3],
    ]);
    header[26] = 0x1000;
    header[27] = h.caps;
    let mut data = b"DDS ".to_vec();
    for v in header {
        data.extend(v.to_le_bytes());
    }
    data.extend(pixels);
    data
}
fn colour_block(a: u16, b: u16, indices: u32) -> Vec<u8> {
    [
        a.to_le_bytes().as_slice(),
        b.to_le_bytes().as_slice(),
        indices.to_le_bytes().as_slice(),
    ]
    .concat()
}
fn decoded(data: &[u8]) -> dds::Texture {
    dds::decode(data, 256 * 1024 * 1024).unwrap()
}
fn fails(data: &[u8], code: &str) {
    match dds::decode(data, 256 * 1024 * 1024) {
        Ok(_) => panic!("Expected {code}"),
        Err(e) => assert_eq!(e.code, code),
    }
}

#[test]
fn upload_preparation_retains_exact_bc_mips_without_rgba_allocation() {
    for (fourcc, stride) in [(*b"DXT1", 8), (*b"DXT3", 16), (*b"DXT5", 16)] {
        let payload: Vec<u8> = (0..7 * stride).map(|v| v as u8).collect();
        let data = source(
            Header {
                width: 12,
                height: 4,
                mips: 4,
                fourcc,
                ..Default::default()
            },
            &payload,
        );
        let prepared = dds::prepare(&data, data.len()).unwrap();
        assert!(prepared.levels.is_empty());
        assert_eq!(prepared.retained_bytes(), data.len());
        let levels = prepared.block_levels().unwrap();
        assert_eq!(
            levels
                .iter()
                .map(|l| (l.width, l.height, l.bytes.len()))
                .collect::<Vec<_>>(),
            [
                (12, 4, 3 * stride),
                (6, 2, 2 * stride),
                (3, 1, stride),
                (1, 1, stride)
            ]
        );
        assert_eq!(
            levels
                .iter()
                .flat_map(|l| l.bytes.iter().copied())
                .collect::<Vec<_>>(),
            payload
        );
        // 3 + 2 + 1 + 1 blocks are required, including two sub-block mip tails.
    }
}

#[test]
fn prepared_storage_enforces_budget_truncation_and_software_fallback_dimensions() {
    let data = source(Header::default(), &colour_block(0xf800, 0, 0));
    assert_eq!(
        dds::prepare(&data, data.len() - 1).err().unwrap().code,
        "dds_limit"
    );
    for length in 0..data.len() {
        assert!(dds::prepare(&data[..length], usize::MAX).is_err());
    }
    let data = source(
        Header {
            width: 3,
            height: 2,
            ..Default::default()
        },
        &colour_block(0xf800, 0, 0),
    );
    let prepared = dds::prepare(&data, 24).unwrap();
    assert!(prepared.encoded.is_none());
    assert_eq!(prepared.levels[0].rgba, [255, 0, 0, 255].repeat(6));
}

#[test]
fn bc1_opaque_interpolation_and_three_colour_transparency() {
    let data = source(Header::default(), &colour_block(0xf800, 0x07e0, 0xe4e4e4e4));
    let t = decoded(&data);
    assert_eq!(
        &t.levels[0].rgba[..16],
        &[
            255, 0, 0, 255, 0, 255, 0, 255, 170, 85, 0, 255, 85, 170, 0, 255
        ]
    );
    let t = decoded(&source(
        Header::default(),
        &colour_block(0, 0xffff, u32::MAX),
    ));
    assert!(t.levels[0].rgba.chunks_exact(4).all(|p| p == [0; 4]));
}
#[test]
fn bc2_alpha_nibbles_and_four_colour_mode_even_when_endpoints_increase() {
    let mut block = vec![0x50, 0xfa, 0x50, 0xfa, 0x50, 0xfa, 0x50, 0xfa];
    block.extend(colour_block(0, 0xffff, 0xffffffff));
    let t = decoded(&source(
        Header {
            fourcc: *b"DXT3",
            ..Default::default()
        },
        &block,
    ));
    assert_eq!(
        &t.levels[0].rgba[..16],
        &[
            170, 170, 170, 0, 170, 170, 170, 85, 170, 170, 170, 170, 170, 170, 170, 255
        ]
    );
}
#[test]
fn bc3_both_alpha_endpoint_orders_match_analytic_tables() {
    for (a, b, expected) in [
        (255, 0, [255, 0, 218, 182, 145, 109, 72, 36]),
        (0, 255, [0, 255, 51, 102, 153, 204, 0, 255]),
    ] {
        let bits = (0..16).fold(0u64, |v, i| v | ((i % 8) as u64) << (i * 3));
        let mut block = vec![a, b];
        block.extend(&bits.to_le_bytes()[..6]);
        block.extend(colour_block(0xf800, 0, 0));
        let t = decoded(&source(
            Header {
                fourcc: *b"DXT5",
                ..Default::default()
            },
            &block,
        ));
        for (i, p) in t.levels[0].rgba.chunks_exact(4).enumerate() {
            assert_eq!(p, [255, 0, 0, expected[i % 8]]);
        }
    }
}
#[test]
fn partial_blocks_and_authored_mips_preserve_dimensions_and_pixels() {
    let mut blocks = colour_block(0xf800, 0, 0);
    blocks.extend(colour_block(0x07e0, 0, 0));
    blocks.extend(colour_block(0x001f, 0, 0));
    let t = decoded(&source(
        Header {
            width: 5,
            height: 3,
            mips: 2,
            ..Default::default()
        },
        &blocks,
    ));
    assert_eq!(
        (
            t.levels[0].width,
            t.levels[0].height,
            t.levels[1].width,
            t.levels[1].height
        ),
        (5, 3, 2, 1)
    );
    assert_eq!(&t.levels[0].rgba[16..20], &[0, 255, 0, 255]);
    assert_eq!(t.levels[1].rgba, [0, 0, 255, 255, 0, 0, 255, 255]);
    assert_eq!(t.info.decoded_bytes, 68);
}
#[test]
fn raw_pitch_channel_masks_and_ignored_xrgb_alpha() {
    let h = Header {
        width: 1,
        height: 2,
        fourcc: [0; 4],
        bits: 32,
        masks: [0xff0000, 0xff00, 0xff, 0xff000000],
        pf: 0x40,
        flags: 0x100f,
        pitch: 8,
        ..Default::default()
    };
    let t = decoded(&source(
        h,
        &[3, 2, 1, 0, 99, 99, 99, 99, 6, 5, 4, 0, 99, 99, 99, 99],
    ));
    assert_eq!(t.levels[0].rgba, [1, 2, 3, 255, 4, 5, 6, 255]);
    assert_eq!(t.levels.len(), 2);
    assert_eq!(t.levels[1].rgba[3], 255);
}
#[test]
fn raw_luminance_alpha_and_565_scaling() {
    let t = decoded(&source(
        Header {
            width: 1,
            height: 1,
            bits: 16,
            masks: [255, 0, 0, 65280],
            pf: 0x20001,
            ..Default::default()
        },
        &[80, 128],
    ));
    assert_eq!(t.levels[0].rgba, [80, 80, 80, 128]);
    let t = decoded(&source(
        Header {
            width: 1,
            height: 1,
            bits: 16,
            masks: [0xf800, 0x07e0, 0x001f, 0],
            pf: 0x40,
            ..Default::default()
        },
        &0x8410_u16.to_le_bytes(),
    ));
    assert_eq!(t.levels[0].rgba, [132, 130, 132, 255]);
}
#[test]
fn generated_mips_average_in_linear_srgb_and_include_odd_edges() {
    let h = Header {
        width: 3,
        height: 1,
        bits: 24,
        masks: [255, 65280, 16711680, 0],
        pf: 0x40,
        ..Default::default()
    };
    let t = decoded(&source(h, &[0, 0, 0, 255, 255, 255, 255, 255, 255]));
    assert_eq!(t.levels.len(), 2);
    assert_eq!(t.levels[1].rgba, [213, 213, 213, 255]);
}

#[test]
fn raw_authored_lower_mips_are_preserved_in_tight_and_dword_aligned_storage() {
    let h = Header {
        width: 2,
        height: 2,
        bits: 24,
        masks: [255, 65280, 16711680, 0],
        pf: 0x40,
        mips: 2,
        ..Default::default()
    };
    let mut pixels = vec![255; 12];
    pixels.extend([255, 0, 0]);
    let t = decoded(&source(h, &pixels));
    assert!(!t.info.generated_mips);
    assert_eq!(t.levels[1].rgba, [255, 0, 0, 255]);
    let h = Header {
        width: 4,
        height: 4,
        bits: 24,
        masks: [255, 65280, 16711680, 0],
        pf: 0x40,
        mips: 3,
        ..Default::default()
    };
    let mut pixels = vec![255; 48];
    pixels.extend([
        0, 255, 0, 0, 255, 0, 99, 99, 0, 255, 0, 0, 255, 0, 99, 99, 0, 0, 255, 99,
    ]);
    let data = source(h, &pixels);
    let t = decoded(&data);
    assert!(
        t.levels[1]
            .rgba
            .chunks_exact(4)
            .all(|p| p == [0, 255, 0, 255])
    );
    assert_eq!(t.levels[2].rgba, [0, 0, 255, 255]);
    assert_eq!(t.info.decoded_bytes, 84);
    fails(&data[..data.len() - 1], "dds_pitch");
    fails(&data[..128 + 48], "dds_truncated");
}
#[test]
fn every_short_prefix_is_rejected_and_decode_budget_is_checked_first() {
    let data = source(Header::default(), &colour_block(0xf800, 0, 0));
    for end in 0..data.len() {
        assert!(dds::decode(&data[..end], 1000).is_err());
    }
    assert_eq!(dds::decode(&data, 63).err().unwrap().code, "dds_limit");
    assert_eq!(dds::inspect(&data).unwrap().decoded_bytes, 64);
}
#[test]
fn malformed_headers_dimensions_surfaces_mips_masks_and_pitch_fail() {
    let block = colour_block(0, 0, 0);
    for (h, code) in [
        (
            Header {
                width: 0,
                ..Default::default()
            },
            "dds_limit",
        ),
        (
            Header {
                width: 8193,
                ..Default::default()
            },
            "dds_limit",
        ),
        (
            Header {
                width: 8192,
                height: 8192,
                ..Default::default()
            },
            "dds_limit",
        ),
        (
            Header {
                caps: 0x200,
                ..Default::default()
            },
            "dds_surface",
        ),
        (
            Header {
                mips: 4,
                ..Default::default()
            },
            "dds_mips",
        ),
        (
            Header {
                fourcc: *b"DX10",
                ..Default::default()
            },
            "dds_format",
        ),
        (
            Header {
                bits: 32,
                masks: [7, 6, 8, 0],
                pf: 0x40,
                ..Default::default()
            },
            "dds_masks",
        ),
        (
            Header {
                bits: 24,
                masks: [5, 0xff00, 0xff0000, 0],
                pf: 0x40,
                ..Default::default()
            },
            "dds_masks",
        ),
        (
            Header {
                bits: 24,
                masks: [255, 65280, 16711680, 0],
                pf: 0x41,
                ..Default::default()
            },
            "dds_masks",
        ),
        (
            Header {
                bits: 32,
                masks: [255, 65280, 16711680, 0],
                pf: 0x40,
                flags: 0x100f,
                pitch: 1,
                ..Default::default()
            },
            "dds_pitch",
        ),
    ] {
        fails(&source(h, &block), code);
    }
}
