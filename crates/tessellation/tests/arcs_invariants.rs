//! Geometry properties shared by clipped, rounded and unlimited arcs joins.
use lyon_path::math::{point, Point};
use lyon_path::Path;
use lyon_tessellation::{
    BuffersBuilder, LineCap, LineJoin, StrokeOptions, StrokeTessellator, StrokeVertex,
    VertexBuffers,
};

type Mesh = VertexBuffers<Point, u32>;

#[test]
fn folded_radial_clip_keeps_round_tip_across_path_transforms() {
    let points = folded_curve();
    for tolerance in [0.02, 0.005, 0.001] {
        for reverse in [false, true] {
            for mirror in [-1.0, 1.0] {
                for offset in [0.0, 1024.0] {
                    for variable in [false, true] {
                        for cap in [LineCap::Butt, LineCap::Square, LineCap::Round] {
                            let mut p = transform(points, mirror, offset);
                            let mut widths = [0.8, 1.0, 1.2];
                            if reverse {
                                p.reverse();
                                widths.reverse();
                            }
                            let path = path(&p, widths);
                            let mut options =
                                options(LineJoin::ArcsRound, 0.5, tolerance).with_line_cap(cap);
                            if variable {
                                options = options.with_variable_line_width(0);
                            }
                            let mesh = mesh(&path, &options);
                            for q in [point(2.088, -0.784), point(0.862, -1.858)] {
                                let probe = point(q.x + offset, q.y * mirror + offset);
                                assert!(contains(&mesh, probe), "missing round tip: {:?}, reverse={}, mirror={}, offset={}, point={:?}", options, reverse, mirror, offset, probe);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn folded_radial_clips_preserve_variable_width_segment_bodies() {
    for reverse in [false, true] {
        let mut points = folded_curve();
        let mut widths = [0.8, 1.0, 1.2];
        if reverse {
            points.reverse();
            widths.reverse();
        }
        let full = path(&points, widths);
        let bodies = [
            path(
                &[
                    points[0], points[1], points[2], points[3], points[3], points[3], points[3],
                ],
                [widths[0], widths[1], widths[1]],
            ),
            path(
                &[
                    points[3], points[4], points[5], points[6], points[6], points[6], points[6],
                ],
                [widths[1], widths[2], widths[2]],
            ),
        ];
        for style in [LineJoin::Arcs, LineJoin::ArcsRound] {
            for tolerance in [0.02, 0.005] {
                for limit in [0.0, 0.5, 0.9] {
                    let options = options(style, limit, tolerance).with_variable_line_width(0);
                    let full = mesh(&full, &options);
                    let bodies = bodies.each_ref().map(|path| mesh(path, &options));
                    for x in -8..=8 {
                        for y in -8..=8 {
                            let probe = point(x as f32 * 0.43 + 0.019, y as f32 * 0.43 + 0.023);
                            if bodies
                                .iter()
                                .any(|body| contains(body, probe) && !near_edge(body, probe))
                            {
                                assert!(
                                    contains(&full, probe),
                                    "body hole: {:?}, reverse={}, point={:?}",
                                    options,
                                    reverse,
                                    probe
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn mixed_radial_clip_keeps_its_tip_after_translation() {
    let points = [
        point(-40.0, 18.630001),
        point(-20.0, 13.823999),
        point(-10.0, 0.0),
        point(0.0, 0.0),
        point(-6.127563, 7.9027195),
        point(2.7080011, 20.0),
        point(-11.744001, 40.0),
    ];
    for reverse in [false, true] {
        for mirror in [-1.0, 1.0] {
            for offset in [0.0, 65536.0] {
                let mut points = transform(points, mirror, offset);
                if reverse {
                    points.reverse();
                }
                let path = path(&points, [1.0; 3]);
                let output = mesh(&path, &options(LineJoin::ArcsRound, 0.5, 0.005));
                let probe = point(2.664 + offset, 0.45 * mirror + offset);
                assert!(
                    contains(&output, probe),
                    "missing mixed-clip tip: reverse={}, mirror={}, offset={}",
                    reverse,
                    mirror,
                    offset
                );
            }
        }
    }
}

#[test]
fn infinite_limit_preserves_finite_unclipped_joins() {
    let mixed = [
        point(-40.0, 0.0),
        point(-20.0, 0.0),
        point(-10.0, 0.0),
        point(0.0, 0.0),
        point(0.0, 10.0),
        point(-15.0, 20.0),
        point(0.0, 40.0),
    ];
    let mut straight = mixed;
    straight[5] = point(0.0, 20.0);
    for points in [mixed, straight, folded_curve()] {
        for reverse in [false, true] {
            let mut points = points;
            if reverse {
                points.reverse();
            }
            let path = path(&points, [1.0; 3]);
            for style in [LineJoin::Arcs, LineJoin::ArcsRound] {
                for variable in [false, true] {
                    let mut options = options(style, f32::MAX, 0.005);
                    if variable {
                        options = options.with_variable_line_width(0);
                    }
                    let finite = mesh(&path, &options);
                    let unlimited = mesh(&path, &options.with_miter_limit(f32::INFINITY));
                    assert_eq!(finite.vertices, unlimited.vertices);
                    assert_eq!(finite.indices, unlimited.indices);
                }
            }
        }
    }
}

#[test]
fn infinite_limit_on_opposite_tangents_uses_finite_round_fallback() {
    for curvature in [0.0, 0.5] {
        let points = [
            point(-40.0, 0.0),
            point(-20.0, curvature),
            point(-10.0, 0.0),
            point(0.0, 0.0),
            point(-10.0, 0.0),
            point(-20.0, curvature),
            point(-40.0, 0.0),
        ];
        let path = path(&points, [1.0; 3]);
        let round = mesh(&path, &options(LineJoin::Round, f32::INFINITY, 0.005));
        for style in [LineJoin::Arcs, LineJoin::ArcsRound] {
            let actual = mesh(&path, &options(style, f32::INFINITY, 0.005));
            assert_eq!(actual.vertices, round.vertices);
            assert_eq!(actual.indices, round.indices);
        }
    }
}

#[test]
fn zero_limit_does_not_grow_a_tip_from_rounding_error() {
    let points = [
        point(-40.0, -18.588),
        point(-20.0, -14.792999),
        point(-10.0, 0.0),
        point(0.0, 0.0),
        point(-9.375342, 3.4789298),
        point(-4.728, 20.0),
        point(0.8319998, 40.0),
    ];
    for reverse in [false, true] {
        for mirror in [-1.0, 1.0] {
            for offset in [0.0, 65536.0] {
                let mut points = transform(points, mirror, offset);
                if reverse {
                    points.reverse();
                }
                let path = path(&points, [1.0; 3]);
                for limit in [0.0, -0.0] {
                    let standard = mesh(&path, &options(LineJoin::Arcs, limit, 0.005));
                    let rounded = mesh(&path, &options(LineJoin::ArcsRound, limit, 0.005));
                    assert_eq!(standard.vertices, rounded.vertices);
                    assert_eq!(standard.indices, rounded.indices);
                }
            }
        }
    }
}

#[test]
fn arcs_preserve_coverage_under_reversal_and_translation() {
    let mut seed = 271829u64;
    let mut random = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((seed >> 32) as u32 % 10001) as f32 / 10000.0
    };
    for case in 0..32 {
        let angle = 0.1 + random() * 2.9;
        let points = [
            point(-40.0, (random() - 0.5) * 60.0),
            point(-20.0, (random() - 0.5) * 30.0),
            point(-10.0, 0.0),
            point(0.0, 0.0),
            point(10.0 * angle.cos(), 10.0 * angle.sin()),
            point((random() - 0.5) * 40.0, 20.0),
            point((random() - 0.5) * 80.0, 40.0),
        ];
        let mut reversed = points;
        reversed.reverse();
        let bodies = [
            path(
                &[
                    points[0], points[1], points[2], points[3], points[3], points[3], points[3],
                ],
                [1.0; 3],
            ),
            path(
                &[
                    points[3], points[4], points[5], points[6], points[6], points[6], points[6],
                ],
                [1.0; 3],
            ),
        ];
        for style in [LineJoin::Arcs, LineJoin::ArcsRound] {
            let options = options(
                style,
                [0.0, 0.5, 1.0, 2.0, 4.0, f32::INFINITY][case % 6],
                0.005,
            );
            let original = mesh(&path(&points, [1.0; 3]), &options);
            let reverse = mesh(&path(&reversed, [1.0; 3]), &options);
            let mut translated = mesh(&path(&transform(points, 1.0, 65536.0), [1.0; 3]), &options);
            for p in &mut translated.vertices {
                p.x -= 65536.0;
                p.y -= 65536.0;
            }
            let bodies = bodies.each_ref().map(|path| mesh(path, &options));
            for _ in 0..96 {
                let probe = point((random() - 0.5) * 20.0, (random() - 0.5) * 20.0);
                if near_edge(&original, probe) {
                    continue;
                }
                for transformed in [&reverse, &translated] {
                    if !near_edge(transformed, probe) {
                        assert_eq!(
                            contains(&original, probe),
                            contains(transformed, probe),
                            "coverage changed: case={}, style={:?}, point={:?}",
                            case,
                            style,
                            probe
                        );
                    }
                }
                if bodies
                    .iter()
                    .any(|body| contains(body, probe) && !near_edge(body, probe))
                {
                    assert!(
                        contains(&original, probe),
                        "segment body lost: case={}, style={:?}, point={:?}",
                        case,
                        style,
                        probe
                    );
                }
            }
        }
    }
}

fn folded_curve() -> [Point; 7] {
    [
        point(-40.0, 6.3240013),
        point(-20.0, 1.9139993),
        point(-10.0, 0.0),
        point(0.0, 0.0),
        point(0.08906084, 9.999603),
        point(-1.0839999, 20.0),
        point(-22.064, 40.0),
    ]
}

fn transform(points: [Point; 7], mirror: f32, offset: f32) -> [Point; 7] {
    points.map(|p| point(p.x + offset, p.y * mirror + offset))
}

fn path(points: &[Point; 7], widths: [f32; 3]) -> Path {
    let mut builder = Path::builder_with_attributes(1);
    builder.begin(points[0], &[widths[0]]);
    builder.cubic_bezier_to(points[1], points[2], points[3], &[widths[1]]);
    builder.cubic_bezier_to(points[4], points[5], points[6], &[widths[2]]);
    builder.end(false);
    builder.build()
}

fn options(style: LineJoin, limit: f32, tolerance: f32) -> StrokeOptions {
    StrokeOptions::default()
        .with_line_width(4.0)
        .with_line_join(style)
        .with_miter_limit(limit)
        .with_tolerance(tolerance)
}

fn mesh(path: &Path, options: &StrokeOptions) -> Mesh {
    let mut mesh = Mesh::new();
    StrokeTessellator::new()
        .tessellate_path(
            path,
            options,
            &mut BuffersBuilder::new(&mut mesh, |mut v: StrokeVertex| {
                let p = v.position();
                let n = v.normal();
                assert!(p.x.is_finite() && p.y.is_finite());
                assert!(n.x.is_finite() && n.y.is_finite());
                assert!(v.line_width().is_finite() && v.advancement().is_finite());
                assert!(v.interpolated_attributes().iter().all(|a| a.is_finite()));
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

// Ignore points close to either triangulation's edges so a change within
// flattening/translation precision is not reported as a coverage regression.
fn near_edge(mesh: &Mesh, p: Point) -> bool {
    mesh.indices.chunks_exact(3).any(|triangle| {
        (0..3).any(|i| {
            let a = mesh.vertices[triangle[i] as usize];
            let b = mesh.vertices[triangle[(i + 1) % 3] as usize];
            let edge = b - a;
            let length = edge.square_length();
            let t = if length == 0.0 {
                0.0
            } else {
                ((p - a).dot(edge) / length).clamp(0.0, 1.0)
            };
            (p - (a + edge * t)).square_length() < 0.03 * 0.03
        })
    })
}
