//! Switching to arcs after a contour has started must keep closure state valid.

use lyon_tessellation::path::builder::{Build, PathBuilder};
use lyon_tessellation::path::math::point;
use lyon_tessellation::{
    BuffersBuilder, LineJoin, StrokeOptions, StrokeTessellator, StrokeVertex, VertexBuffers,
};

#[test]
fn enabling_arcs_mid_contour_can_close_and_reuse_the_builder() {
    for join in [LineJoin::Arcs, LineJoin::ArcsRound] {
        for variable_width in [false, true] {
            for switch_after in 0..=3 {
                let mut options = StrokeOptions::default().with_line_width(2.0);
                if variable_width {
                    options = options.with_variable_line_width(0);
                }
                let mut mesh: VertexBuffers<_, u32> = VertexBuffers::new();
                let mut tessellator = StrokeTessellator::new();
                let mut output = BuffersBuilder::new(&mut mesh, |v: StrokeVertex| v.position());
                let mut builder = tessellator.builder_with_attributes(1, &options, &mut output);
                for offset in [0.0, 20.0] {
                    builder.set_line_join(LineJoin::Miter);
                    builder.begin(point(offset, 0.0), &[1.0]);
                    let points = [
                        point(offset + 10.0, 0.0),
                        point(offset + 10.0, 10.0),
                        point(offset, 10.0),
                    ];
                    for (index, position) in points.iter().copied().enumerate() {
                        if index == switch_after {
                            builder.set_line_join(join);
                        }
                        builder.line_to(position, &[1.0 + index as f32 * 0.1]);
                    }
                    if switch_after == points.len() {
                        builder.set_line_join(join);
                    }
                    builder.end(true);
                }
                builder.build().unwrap();
                assert!(!mesh.indices.is_empty());
                assert!(mesh
                    .vertices
                    .iter()
                    .all(|p| p.x.is_finite() && p.y.is_finite()));
                assert!(mesh
                    .indices
                    .iter()
                    .all(|&i| (i as usize) < mesh.vertices.len()));
            }
        }
    }
}
