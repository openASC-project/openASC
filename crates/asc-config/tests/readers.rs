use asc_config::{parse_ini, parse_lut, read_text};

#[test]
fn ini_preserves_values_order_and_source_lines_without_path_escaping() {
    let ini = parse_ini("\u{feff}# original fixture\n[model_7] ; comment\nfile=\"folder\\a;b.kn5\" ; tail\nempty=\nurl=https://example.org/a#part\nname=driver's car\nvalue=a=b\n[second]\nx=2", "original.ini").unwrap();
    assert_eq!(ini.sections.len(), 2);
    let section = &ini.sections[0];
    assert_eq!((&section.name, section.line), (&"MODEL_7".to_string(), 2));
    assert_eq!(section.get("file").unwrap().text, "folder\\a;b.kn5");
    assert_eq!(section.get("file").unwrap().line, 3);
    assert_eq!(section.get("EMPTY").unwrap().text, "");
    assert_eq!(
        section.get("URL").unwrap().text,
        "https://example.org/a#part"
    );
    assert_eq!(section.get("NAME").unwrap().text, "driver's car");
    assert_eq!(section.get("VALUE").unwrap().text, "a=b");
    assert_eq!(ini.sections[1].name, "SECOND");
}
#[test]
fn ambiguous_ini_input_reports_the_exact_source_and_line() {
    for (text, code, line) in [
        ("x=1", "missing_section", 1),
        ("[]", "empty_section", 1),
        ("[A]junk", "invalid_section", 1),
        ("[", "invalid_section", 1),
        ("[[A]]", "invalid_section", 1),
        ("[A]\n[a]", "duplicate_section", 2),
        ("[A]\nx=1\nX=2", "duplicate_key", 3),
        ("[A]\ninvalid", "invalid_assignment", 2),
        ("[A]\n=1", "invalid_key", 2),
        ("[A]\nbad key=1", "invalid_key", 2),
        ("[A]\nx=\"unfinished", "unclosed_quote", 2),
        ("[A]\nx=\"a\" \"b\"", "invalid_quote", 2),
        ("[A]\nx=a\"b\"", "invalid_quote", 2),
    ] {
        let error = parse_ini(text, "broken.ini").unwrap_err();
        assert_eq!(
            (error.code, error.source.as_str(), error.line),
            (code, "broken.ini", line),
            "{text}"
        );
    }
}
#[test]
fn text_bytes_records_and_encoding_are_bounded() {
    assert_eq!(
        parse_ini(&" ".repeat(4 * 1024 * 1024 + 1), "limit")
            .unwrap_err()
            .code,
        "text_limit"
    );
    assert_eq!(
        parse_ini(&"[A]\n".repeat(100_001), "limit")
            .unwrap_err()
            .code,
        "text_limit"
    );
    let path = std::env::temp_dir().join(format!("openasc-encoding-{}.ini", std::process::id()));
    std::fs::write(&path, b"[A]\nNAME=caf\xe9").unwrap();
    let error = read_text(&path).unwrap_err();
    std::fs::remove_file(path).unwrap();
    assert_eq!(error.code, "text_encoding");
}
#[test]
fn lut_interpolates_clamps_and_handles_single_points() {
    let table = parse_lut(
        "// original fixture\n0|10\n10|30 ; comment\n30|-10",
        "curve.lut",
    )
    .unwrap();
    assert_eq!(table.points(), &[[0., 10.], [10., 30.], [30., -10.]]);
    for (x, y) in [
        (-5., 10.),
        (0., 10.),
        (5., 20.),
        (10., 30.),
        (20., 10.),
        (30., -10.),
        (99., -10.),
    ] {
        assert_eq!(table.sample(x).unwrap(), y);
    }
    let one = parse_lut("1e2|-2.5e-1", "single").unwrap();
    assert_eq!(one.sample(-1e6).unwrap(), -0.25);
    assert_eq!(one.sample(1e6).unwrap(), -0.25);
}
#[test]
fn lut_rejects_malformed_unordered_nonfinite_and_overflowing_input() {
    for (text, code, line) in [
        ("", "empty_lut", 0),
        ("0|1|2", "invalid_lut_row", 1),
        ("0|1\n0|2", "unordered_lut", 2),
        ("1|1\n0|2", "unordered_lut", 2),
        ("a|1", "invalid_number", 1),
        ("1|nan", "nonfinite_number", 1),
        ("inf|1", "nonfinite_number", 1),
        ("1|", "invalid_number", 1),
    ] {
        let error = parse_lut(text, "curve.lut").err().unwrap();
        assert_eq!((error.code, error.line), (code, line));
    }
    let table = parse_lut("-1e308|0\n1e308|1", "wide").unwrap();
    assert_eq!(table.sample(0.).unwrap_err().code, "numeric_range");
    for x in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(table.sample(x).unwrap_err().code, "nonfinite_number");
    }
}
