//! Compatibility checks for opt-in joins and existing stroke entry points.

use lyon_path::math::{point, Point, Vector};
use lyon_path::Path;
use lyon_tessellation::{
    BuffersBuilder, LineJoin, Side, StrokeOptions, StrokeTessellator, StrokeVertex, VertexBuffers,
    VertexSource,
};

#[test]
fn legacy_minimum_constant_keeps_existing_styles_at_one() {
    let options = StrokeOptions::default().with_miter_limit(StrokeOptions::MINIMUM_MITER_LIMIT);
    assert_eq!(options.miter_limit, 1.0);
}

#[test]
fn sub_unit_arcs_limits_do_not_depend_on_option_setter_order() {
    for join in [LineJoin::Arcs, LineJoin::ArcsRound] {
        for limit in [0.0, 0.5, 1.0] {
            let join_first = StrokeOptions::default()
                .with_line_join(join)
                .with_miter_limit(limit);
            let limit_first = StrokeOptions::default()
                .with_miter_limit(limit)
                .with_line_join(join);
            assert_eq!(join_first, limit_first);
            assert_eq!(join_first.miter_limit, limit);
        }
    }
}

#[cfg(feature = "serialization")]
#[test]
fn legacy_join_serde_indices_and_json_names_are_unchanged() {
    use serde::de::value::{Error, U32Deserializer};
    use serde::Deserialize;

    // Binary formats pass these enum indices to Serde. Appending variants must
    // not reinterpret an existing index as a different join.
    for (index, name, join) in [
        (0, "Miter", LineJoin::Miter),
        (1, "MiterClip", LineJoin::MiterClip),
        (2, "Round", LineJoin::Round),
        (3, "Bevel", LineJoin::Bevel),
    ] {
        let decoded = LineJoin::deserialize(U32Deserializer::<Error>::new(index)).unwrap();
        assert_eq!(decoded, join);
        assert_eq!(serde_json::to_value(join).unwrap(), name);
        assert_eq!(
            serde_json::from_value::<LineJoin>(name.into()).unwrap(),
            join
        );
    }
}

#[test]
fn path_entry_points_preserve_mesh_and_vertex_metadata() {
    for join in [
        LineJoin::Miter,
        LineJoin::MiterClip,
        LineJoin::Round,
        LineJoin::Bevel,
        LineJoin::Arcs,
        LineJoin::ArcsRound,
    ] {
        for closed in [false, true] {
            for attributes in [false, true] {
                for variable_width in [false, true] {
                    if variable_width && !attributes {
                        continue;
                    }
                    let path = sample_path(attributes, closed);
                    let mut options = StrokeOptions::default()
                        .with_line_join(join)
                        .with_line_width(4.0)
                        .with_miter_limit(1.6)
                        .with_tolerance(0.05);
                    if variable_width {
                        options = options.with_variable_line_width(0);
                    }
                    let mut path_output: VertexBuffers<VertexSnapshot, u32> = VertexBuffers::new();
                    let mut direct_output: VertexBuffers<VertexSnapshot, u32> =
                        VertexBuffers::new();
                    StrokeTessellator::new()
                        .tessellate_path(
                            &path,
                            &options,
                            &mut BuffersBuilder::new(&mut path_output, snapshot),
                        )
                        .unwrap();
                    let mut direct = StrokeTessellator::new();
                    if attributes {
                        direct
                            .tessellate_with_ids(
                                path.id_iter(),
                                &path,
                                Some(&path),
                                &options,
                                &mut BuffersBuilder::new(&mut direct_output, snapshot),
                            )
                            .unwrap();
                    } else {
                        direct
                            .tessellate(
                                path.iter(),
                                &options,
                                &mut BuffersBuilder::new(&mut direct_output, snapshot),
                            )
                            .unwrap();
                    }
                    assert_eq!(path_output.vertices, direct_output.vertices, "{:?}", join);
                    assert_eq!(path_output.indices, direct_output.indices, "{:?}", join);
                }
            }
        }
    }
}

#[test]
fn switching_joins_on_a_reused_tessellator_matches_fresh_tessellators() {
    let mut reused = StrokeTessellator::new();
    for closed in [false, true] {
        for (attributes, variable_width) in [(false, false), (true, false), (true, true)] {
            let path = sample_path(attributes, closed);
            for join in [
                LineJoin::Arcs,
                LineJoin::Round,
                LineJoin::ArcsRound,
                LineJoin::Miter,
                LineJoin::Arcs,
            ] {
                let mut options = StrokeOptions::default()
                    .with_line_join(join)
                    .with_line_width(4.0)
                    .with_miter_limit(1.6)
                    .with_tolerance(0.05);
                if variable_width {
                    options = options.with_variable_line_width(0);
                }
                let mut actual: VertexBuffers<VertexSnapshot, u32> = VertexBuffers::new();
                let mut expected: VertexBuffers<VertexSnapshot, u32> = VertexBuffers::new();
                let mut fresh = StrokeTessellator::new();
                reused
                    .tessellate_path(
                        &path,
                        &options,
                        &mut BuffersBuilder::new(&mut actual, snapshot),
                    )
                    .unwrap();
                fresh
                    .tessellate_path(
                        &path,
                        &options,
                        &mut BuffersBuilder::new(&mut expected, snapshot),
                    )
                    .unwrap();
                assert_eq!(actual.vertices, expected.vertices, "{:?}", join);
                assert_eq!(actual.indices, expected.indices, "{:?}", join);
            }
        }
    }
}

#[derive(Debug, PartialEq)]
struct VertexSnapshot {
    position: Point,
    normal: Vector,
    position_on_path: Point,
    line_width: f32,
    advancement: f32,
    side: Side,
    source: VertexSource,
    attributes: Vec<f32>,
}

fn snapshot(mut vertex: StrokeVertex) -> VertexSnapshot {
    VertexSnapshot {
        position: vertex.position(),
        normal: vertex.normal(),
        position_on_path: vertex.position_on_path(),
        line_width: vertex.line_width(),
        advancement: vertex.advancement(),
        side: vertex.side(),
        source: vertex.source(),
        attributes: vertex.interpolated_attributes().to_vec(),
    }
}

fn sample_path(attributes: bool, closed: bool) -> Path {
    let mut builder = Path::builder_with_attributes(if attributes { 2 } else { 0 });
    let a: &[f32] = if attributes { &[1.0, 5.0] } else { &[] };
    let b: &[f32] = if attributes { &[1.5, 8.0] } else { &[] };
    builder.begin(point(-40.0, 0.0), a);
    builder.cubic_bezier_to(point(-20.0, 0.0), point(-10.0, 0.0), point(0.0, 0.0), b);
    builder.cubic_bezier_to(point(0.0, 10.0), point(60.0, 20.0), point(0.0, 40.0), a);
    builder.quadratic_bezier_to(point(-20.0, 60.0), point(-40.0, 30.0), b);
    builder.line_to(point(-45.0, 15.0), a);
    builder.end(closed);
    builder.build()
}
