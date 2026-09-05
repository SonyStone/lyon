use lyon_tessellation::path::{
    math::{point, Point},
    Path,
};
use lyon_tessellation::{
    BuffersBuilder, LineCap, LineJoin, StrokeOptions, StrokeTessellator, StrokeVertex,
    VertexBuffers,
};

#[test]
fn sharp_turn_between_flattened_samples_has_valid_attachments() {
    for close in [false, true] {
        let mut builder = Path::builder_with_attributes(1);
        builder.begin(point(25.769997, 31.230003), &[0.2]);
        builder.cubic_bezier_to(
            point(12.080002, 7.6399994),
            point(47.949997, -44.0),
            point(12.889999, 48.11),
            &[3.0],
        );
        builder.quadratic_bezier_to(point(40.629997, -45.75), point(-42.53, 36.199997), &[0.7]);
        builder.cubic_bezier_to(
            point(-33.18, 41.160004),
            point(43.449997, 36.809998),
            point(18.46, 39.1),
            &[0.2],
        );
        builder.end(close);
        check_styles_and_caps(&builder.build(), 0.1, 0.01);
    }
}

#[test]
fn collapsed_offset_edges_have_finite_join_normals() {
    // The taper makes the offset endpoints coincide in f32, while the
    // centerline endpoints remain distinct. Reverse to exercise either edge.
    let points = [
        point(65536.0, 65536.0),
        point(65537.0, 65536.0),
        point(65538.0, 65536.5),
    ];
    let widths = [2.0, 4.0, 4.0];
    for reverse in [false, true] {
        let indices = if reverse { [2, 1, 0] } else { [0, 1, 2] };
        let mut builder = Path::builder_with_attributes(1);
        builder.begin(points[indices[0]], &[widths[indices[0]]]);
        for i in &indices[1..] {
            builder.line_to(points[*i], &[widths[*i]]);
        }
        builder.end(false);
        check_styles_and_caps(&builder.build(), 1.0, 0.01);
    }
}

#[test]
fn merged_curve_endpoint_has_valid_cap_attachments() {
    let mut builder = Path::builder_with_attributes(1);
    builder.begin(point(3722.0002, 631.0), &[0.2]);
    builder.cubic_bezier_to(
        point(2026.0, -3044.0),
        point(5907.9995, -3438.0),
        point(-802.0, 5168.9995),
        &[3.0],
    );
    builder.quadratic_bezier_to(point(1283.0, 94.00006), point(3957.0002, 3556.0), &[0.7]);
    builder.cubic_bezier_to(
        point(-1596.0, 4554.0),
        point(-528.0, -464.00012),
        point(-600.0001, 174.0),
        &[0.2],
    );
    builder.end(false);
    check_styles_and_caps(&builder.build(), 2000.0, 100.0);
}

fn check_styles_and_caps(path: &Path, width: f32, tolerance: f32) {
    let mut reused = StrokeTessellator::new();
    for style in [
        LineJoin::Miter,
        LineJoin::MiterClip,
        LineJoin::Bevel,
        LineJoin::Round,
        LineJoin::Arcs,
        LineJoin::ArcsRound,
    ] {
        for cap in [LineCap::Butt, LineCap::Square, LineCap::Round] {
            let options = StrokeOptions::default()
                .with_line_join(style)
                .with_line_cap(cap)
                .with_variable_line_width(0)
                .with_line_width(width)
                .with_tolerance(tolerance);
            let mesh = checked_mesh(&mut reused, path, &options);
            let fresh = checked_mesh(&mut StrokeTessellator::new(), path, &options);
            assert_eq!(mesh.vertices, fresh.vertices);
            assert_eq!(mesh.indices, fresh.indices);
        }
    }
}

fn checked_mesh(
    tessellator: &mut StrokeTessellator,
    path: &Path,
    options: &StrokeOptions,
) -> VertexBuffers<Point, u32> {
    let mut mesh = VertexBuffers::new();
    tessellator
        .tessellate_path(
            path,
            options,
            &mut BuffersBuilder::new(&mut mesh, |mut v: StrokeVertex| {
                let p = v.position();
                let n = v.normal();
                assert!(p.x.is_finite() && p.y.is_finite(), "{:?}: {:?}", options, p);
                assert!(n.x.is_finite() && n.y.is_finite(), "{:?}: {:?}", options, n);
                assert!(v.line_width().is_finite());
                assert!(v.advancement().is_finite());
                assert!(v.interpolated_attributes()[0].is_finite());
                p
            }),
        )
        .unwrap();
    assert!(!mesh.indices.is_empty());
    assert!(mesh
        .indices
        .iter()
        .all(|&i| (i as usize) < mesh.vertices.len()));
    mesh
}
