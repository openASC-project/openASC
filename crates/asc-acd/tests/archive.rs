use asc_acd::{Archive, folder_key};

fn encoded(files: &[(&str, &[u8])], header: Option<i32>) -> Vec<u8> {
    let mut out = Vec::new();
    let key = folder_key("fixture").unwrap();
    if let Some(value) = header {
        out.extend((-1111i32).to_le_bytes());
        out.extend(value.to_le_bytes());
    }
    for (name, data) in files {
        out.extend((name.len() as i32).to_le_bytes());
        out.extend(name.as_bytes());
        out.extend((data.len() as i32).to_le_bytes());
        for (i, &v) in data.iter().enumerate() {
            out.extend([v.wrapping_add(key[i % key.len()]), 11, 22, 33]);
        }
    }
    out
}
#[test]
fn independent_key_vectors_cover_short_names_case_and_signed_wrapping() {
    for (name, expected) in [
        ("a", "97-0-0-131-66-101-171-171"),
        ("AB", "195-158-0-33-66-101-171-99"),
        ("abcdef", "85-104-132-143-246-161-74-103"),
        ("abcdefghij", "247-134-32-237-238-213-74-107"),
        ("bmw_m3_e30", "108-96-216-121-166-192-73-49"),
    ] {
        assert_eq!(folder_key(name).unwrap(), expected.as_bytes());
    }
    assert_eq!(
        folder_key(&"z".repeat(128)).unwrap(),
        b"0-202-207-253-44-95-49-123"
    );
    for name in ["", "../a", "car name", "nonascii-\u{e9}", &"a".repeat(129)] {
        assert_eq!(folder_key(name).unwrap_err().code, "acd_identifier");
    }
}
#[test]
fn fixed_encoded_record_decodes_without_using_the_fixture_encoder() {
    let mut bytes = 7i32.to_le_bytes().to_vec();
    bytes.extend(b"car.ini");
    bytes.extend(2i32.to_le_bytes());
    bytes.extend([140, 0, 0, 0, 145, 0, 0, 0]);
    let archive = Archive::parse(bytes, "ab", "fixed.acd").unwrap();
    assert_eq!(archive.read("CAR.INI").unwrap(), b"[X");
}
#[test]
fn record_keys_reset_and_padding_is_not_mistaken_for_text() {
    let all: Vec<_> = (0..=255).collect();
    let files = [
        ("first.ini", all.as_slice()),
        ("second.lut", b"0|1\n1|2".as_slice()),
        ("empty.txt", b"".as_slice()),
    ];
    for header in [None, Some(42)] {
        let archive = Archive::parse(encoded(&files, header), "fixture", "original.acd").unwrap();
        assert_eq!(archive.header, header);
        for (name, data) in files {
            assert_eq!(archive.read(&name.to_uppercase()).unwrap(), data);
        }
        assert_eq!(
            archive.read("missing.ini").unwrap_err().code,
            "car_data_missing"
        );
        assert_eq!(
            archive.read("../first.ini").unwrap_err().code,
            "car_data_name"
        );
    }
}
#[test]
fn every_truncated_prefix_fails_with_an_in_bounds_source_offset() {
    let data = encoded(&[("a.ini", b"original")], Some(42));
    for end in 0..data.len() {
        let error = Archive::parse(data[..end].to_vec(), "fixture", "truncated.acd")
            .err()
            .unwrap();
        assert_eq!(error.source, "truncated.acd");
        assert!(error.byte_offset <= end);
    }
    let mut trailing = data;
    trailing.push(1);
    assert_eq!(
        Archive::parse(trailing, "fixture", "original")
            .err()
            .unwrap()
            .code,
        "acd_truncated"
    );
}
#[test]
fn invalid_names_duplicate_records_and_declared_lengths_fail_before_allocation() {
    for name in [
        "../bad.ini",
        "/bad.ini",
        "folder/file.ini",
        "C:bad.ini",
        ".hidden",
        "bad name.ini",
    ] {
        assert_eq!(
            Archive::parse(encoded(&[(name, b"")], None), "fixture", "names")
                .err()
                .unwrap()
                .code,
            "acd_name"
        );
    }
    assert_eq!(
        Archive::parse(
            encoded(&[("same.ini", b"a"), ("SAME.INI", b"b")], None),
            "fixture",
            "duplicate"
        )
        .err()
        .unwrap()
        .code,
        "acd_duplicate"
    );
    for length in [-1i32, 0, 256] {
        assert_eq!(
            Archive::parse(length.to_le_bytes().to_vec(), "fixture", "name")
                .err()
                .unwrap()
                .code,
            "acd_name"
        );
    }
    for length in [-1, asc_acd::MAX_ENTRY as i32 + 1] {
        let mut bytes = 1i32.to_le_bytes().to_vec();
        bytes.push(b'a');
        bytes.extend(length.to_le_bytes());
        assert_eq!(
            Archive::parse(bytes, "fixture", "limit")
                .err()
                .unwrap()
                .code,
            "acd_limit"
        );
    }
}
#[test]
fn archive_and_entry_count_budgets_are_enforced() {
    assert_eq!(
        Archive::open(std::path::Path::new("."), "fixture", "directory")
            .err()
            .unwrap()
            .code,
        "acd_io"
    );
    assert_eq!(
        Archive::parse(vec![0; asc_acd::MAX_ARCHIVE + 1], "fixture", "limit")
            .err()
            .unwrap()
            .code,
        "acd_limit"
    );
    let names: Vec<_> = (0..=asc_acd::MAX_ENTRIES)
        .map(|i| format!("file{i}.ini"))
        .collect();
    let files: Vec<_> = names.iter().map(|n| (n.as_str(), b"".as_slice())).collect();
    assert_eq!(
        Archive::parse(encoded(&files, None), "fixture", "count")
            .err()
            .unwrap()
            .code,
        "acd_limit"
    );
}
