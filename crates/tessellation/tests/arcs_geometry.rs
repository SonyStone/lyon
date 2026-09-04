//! Arcs joins must preserve coverage near straight curves and tight miter cuts.
use lyon_path::{
    math::{point, Point},
    Path,
};
use lyon_tessellation::{
    BuffersBuilder, LineJoin, StrokeOptions, StrokeTessellator, StrokeVertex, VertexBuffers,
};

type Mesh = VertexBuffers<Point, u32>;

#[test]
fn nearly_straight_curve_keeps_the_arcs_join() {
    let probe = point(1.6, -1.7);
    for bend in [0.0, 0.0000015, -0.0000015] {
        let mut path = Path::builder();
        path.begin(point(-40.0, 0.0));
        path.cubic_bezier_to(point(-20.0, bend), point(-10.0, 0.0), point(0.0, 0.0));
        path.cubic_bezier_to(point(0.0, 10.0), point(-15.0, 20.0), point(0.0, 40.0));
        path.end(false);
        let output = mesh(&path.build(), LineJoin::Arcs, 100.0);
        assert!(
            contains(&output, probe),
            "arcs join lost coverage at {:?} after control-point bend {}",
            probe,
            bend
        );
    }
}

#[test]
fn subunit_miter_limits_preserve_straight_segment_bodies() {
    let mut path = Path::builder();
    path.begin(point(-40.0, 0.0));
    path.line_to(point(0.0, 0.0));
    path.line_to(point(0.0, 40.0));
    path.end(false);
    let path = path.build();
    let mut failures = Vec::new();
    for join in [LineJoin::MiterClip, LineJoin::Arcs] {
        for limit in [0.0, 0.5, 0.75, 0.99] {
            let output = mesh(&path, join, limit);
            for probe in [point(-0.1, -1.9), point(1.9, 0.1)] {
                if !contains(&output, probe) {
                    failures.push((join, limit, probe));
                }
            }
            assert!(!contains(&output, point(1.8, -1.8)), "miter cut refilled");
        }
    }
    assert!(
        failures.is_empty(),
        "lost segment-body coverage: {:?}",
        failures
    );
}

#[test]
fn subunit_miter_clipping_preserves_bodies_across_turns_and_widths() {
    for angle in [0.3_f32, 1.0, 1.57, 2.2, 2.8] {
        for mirror in [-1.0, 1.0] {
            let points = [
                point(-40.0, 0.0),
                point(0.0, 0.0),
                point(40.0 * angle.cos(), mirror * 40.0 * angle.sin()),
            ];
            for widths in [[1.0, 1.0, 1.0], [0.8, 1.0, 1.2]] {
                for reverse in [false, true] {
                    let mut points = points;
                    let mut widths = widths;
                    if reverse {
                        points.reverse();
                        widths.reverse();
                    }
                    let full = attributed_path(&points, &widths);
                    let segments = [
                        attributed_path(&points[..2], &widths[..2]),
                        attributed_path(&points[1..], &widths[1..]),
                    ];
                    for join in [LineJoin::MiterClip, LineJoin::Arcs, LineJoin::ArcsRound] {
                        for limit in [0.0, 0.5, 0.9] {
                            let options = StrokeOptions::default()
                                .with_line_width(4.0)
                                .with_line_join(join)
                                .with_miter_limit(limit)
                                .with_variable_line_width(0)
                                .with_tolerance(0.001);
                            let full = mesh_with_options(&full, &options);
                            let bodies =
                                segments.each_ref().map(|p| mesh_with_options(p, &options));
                            for x in -8..=8 {
                                for y in -8..=8 {
                                    let p = point(x as f32 * 0.43 + 0.019, y as f32 * 0.43 + 0.023);
                                    if bodies.iter().any(|body| contains(body, p)) {
                                        assert!(contains(&full, p),
                                            "body hole: {:?}, limit {}, angle {}, mirror {}, reverse {}, widths {:?}, point {:?}",
                                            join, limit, angle, mirror, reverse, widths, p);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

fn attributed_path(points: &[Point], widths: &[f32]) -> Path {
    let mut path = Path::builder_with_attributes(1);
    path.begin(points[0], &[widths[0]]);
    for (&p, &width) in points.iter().zip(widths).skip(1) {
        path.line_to(p, &[width]);
    }
    path.end(false);
    path.build()
}

fn mesh(path: &Path, join: LineJoin, limit: f32) -> Mesh {
    mesh_with_options(
        path,
        &StrokeOptions::default()
            .with_line_join(join)
            .with_line_width(4.0)
            .with_miter_limit(limit)
            .with_tolerance(0.001),
    )
}

fn mesh_with_options(path: &Path, options: &StrokeOptions) -> Mesh {
    let mut result = Mesh::new();
    StrokeTessellator::new()
        .tessellate_path(
            path,
            options,
            &mut BuffersBuilder::new(&mut result, |v: StrokeVertex| v.position()),
        )
        .unwrap();
    result
}

fn contains(mesh: &Mesh, p: Point) -> bool {
    mesh.indices.chunks_exact(3).any(|t| {
        let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.vertices[i as usize]);
        let signs = [
            (b - a).cross(p - a),
            (c - b).cross(p - b),
            (a - c).cross(p - c),
        ];
        (b - a).cross(c - a).abs() > 1.0e-8
            && (signs.iter().all(|s| *s >= 0.0) || signs.iter().all(|s| *s <= 0.0))
    })
}
