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

#[test]
fn output_errors_abort_instead_of_becoming_round_fallbacks() {
    use lyon_tessellation::path::Path;
    use lyon_tessellation::{GeometryBuilderError, TessellationError};

    let mut path = Path::builder();
    path.begin(point(-40.0, 0.0));
    path.cubic_bezier_to(point(-20.0, 0.0), point(-10.0, 0.0), point(0.0, 0.0));
    path.cubic_bezier_to(point(0.0, 10.0), point(60.0, 20.0), point(0.0, 40.0));
    path.end(false);
    let path = path.build();
    for join in [LineJoin::Arcs, LineJoin::ArcsRound] {
        let options = StrokeOptions::default()
            .with_line_join(join)
            .with_line_width(4.0)
            .with_miter_limit(1.5);
        let mut complete = FailingOutput::new(u32::MAX);
        StrokeTessellator::new()
            .tessellate_path(&path, &options, &mut complete)
            .unwrap();
        // Fail each insertion, including base, Arcs mesh, rounded tip and caps.
        for fail_at in 0..complete.vertices {
            let mut output = FailingOutput::new(fail_at);
            let result = StrokeTessellator::new().tessellate_path(&path, &options, &mut output);
            assert!(matches!(
                result,
                Err(TessellationError::GeometryBuilder(
                    GeometryBuilderError::InvalidVertex
                ))
            ));
            assert!(output.aborted);
        }
    }
}

struct FailingOutput {
    vertices: u32,
    fail_at: u32,
    aborted: bool,
}

impl FailingOutput {
    fn new(fail_at: u32) -> Self {
        Self {
            vertices: 0,
            fail_at,
            aborted: false,
        }
    }
}

impl lyon_tessellation::GeometryBuilder for FailingOutput {
    fn add_triangle(
        &mut self,
        _: lyon_tessellation::VertexId,
        _: lyon_tessellation::VertexId,
        _: lyon_tessellation::VertexId,
    ) {
    }
    fn abort_geometry(&mut self) {
        self.aborted = true;
    }
}

impl lyon_tessellation::StrokeGeometryBuilder for FailingOutput {
    fn add_stroke_vertex(
        &mut self,
        _: StrokeVertex,
    ) -> Result<lyon_tessellation::VertexId, lyon_tessellation::GeometryBuilderError> {
        if self.vertices >= self.fail_at {
            return Err(lyon_tessellation::GeometryBuilderError::InvalidVertex);
        }
        let id = lyon_tessellation::VertexId(self.vertices);
        self.vertices += 1;
        Ok(id)
    }
}
