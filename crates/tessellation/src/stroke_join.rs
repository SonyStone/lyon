//! Prepares and emits endpoint joins and sharp turns between flattened samples.
//! The path walker supplies neighbors and derivatives; mesh topology stays here.

use super::*;
use crate::stroke_arcs::{
    self, JoinConstruction, JoinInput, ParallelRectangleJoin, Point64, RadialClipJoin,
    ResolvedArcsJoin, SegmentEnd, TurnSide, Vector64,
};
use crate::stroke_arcs_mesh::{ArcsMesh, ArcsOutput, ValidatedFan};

#[path = "stroke_round_clip.rs"]
mod round_clip;

#[derive(Default)]
pub(super) struct JoinWorkspace {
    mesh: ArcsMesh,
    vertex_ids: Vec<VertexId>,
    #[cfg(test)]
    last_fallback: Option<ArcsJoinFallback>,
}

impl JoinWorkspace {
    // Keep the cause until the adapter chooses Round. Tests inspect this without
    // adding a diagnostic field or callback to the public stroke API.
    fn fallback(&mut self, reason: ArcsJoinFallback) -> LineJoin {
        #[cfg(test)]
        {
            self.last_fallback = Some(reason);
        }
        #[cfg(not(test))]
        {
            let _ = reason;
        }
        LineJoin::Round
    }
}

#[derive(Debug, PartialEq, Eq)]
enum ArcsJoinFallback {
    MissingDifferentials,
    Construction(stroke_arcs::JoinError),
    UnrepresentablePoint,
    Mesh(crate::stroke_arcs_mesh::ArcsMeshError),
    IncompatibleAttachments,
    CollapsedVertex,
}

#[allow(clippy::large_enum_variant)]
enum PreparedJoin {
    Standard(LineJoin),
    Arcs(PreparedArcsJoin),
}

enum JoinEmission {
    Emitted,
    Fallback(ArcsJoinFallback),
}

impl JoinEmission {
    fn emitted(self, workspace: &mut JoinWorkspace) -> bool {
        match self {
            Self::Emitted => true,
            Self::Fallback(reason) => {
                workspace.fallback(reason);
                false
            }
        }
    }
}

#[allow(clippy::large_enum_variant)] // Join preparation must not allocate in the stroke hot path.
enum PreparedArcsJoin {
    Curved(ResolvedArcsJoin),
    ParallelRectangle(PreparedParallelRectangle),
    RadialClip(PreparedRadialClip),
}

#[cfg(not(test))]
const _: () = assert!(core::mem::size_of::<PreparedArcsJoin>() <= 152);

#[derive(Copy, Clone)]
struct PreparedParallelRectangle {
    near_left: Point,
    near_right: Point,
    far_left: Point,
    far_right: Point,
}

#[derive(Copy, Clone)]
struct PreparedRadialClip {
    turn: TurnSide,
    incoming_offset_point: Point,
    outgoing_offset_point: Point,
    incoming: Point,
    outgoing: Point,
}

/// Process an original endpoint or a flattened sample that needs a full join.
#[allow(clippy::too_many_arguments)]
pub(super) fn tessellate_endpoint_join<const FIXED_WIDTH: bool>(
    join: &mut EndpointData,
    neighbors: [&EndpointData; 2],
    differentials: &history::DifferentialHistory,
    emit_edge: bool,
    options: &StrokeOptions,
    arcs: &mut JoinWorkspace,
    vertex: &mut StrokeVertexData,
    attributes: &dyn AttributeStore,
    output: &mut dyn StrokeGeometryBuilder,
) -> Result<(), TessellationError> {
    if matches!(join.line_join, LineJoin::Arcs | LineJoin::ArcsRound)
        || (join.line_join == LineJoin::MiterClip && options.miter_limit < 1.0)
    {
        return tessellate_extended_join::<FIXED_WIDTH>(
            join,
            neighbors,
            differentials,
            emit_edge,
            options,
            arcs,
            vertex,
            attributes,
            output,
        );
    }
    compute_join_positions::<FIXED_WIDTH>(join, neighbors, options, vertex)?;
    emit_join_base::<FIXED_WIDTH>(join, neighbors[0], emit_edge, vertex, attributes, output)?;
    tessellate_join::<false>(
        join,
        neighbors,
        join.line_join,
        None,
        options,
        arcs,
        vertex,
        attributes,
        output,
    )
}

// Outlining matters even when Arcs is disabled: its large preparation/mesh
// temporaries must not enlarge the stack frame of every flattening step.
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn tessellate_extended_join<const FIXED_WIDTH: bool>(
    join: &mut EndpointData,
    neighbors: [&EndpointData; 2],
    differentials: &history::DifferentialHistory,
    emit_edge: bool,
    options: &StrokeOptions,
    arcs: &mut JoinWorkspace,
    vertex: &mut StrokeVertexData,
    attributes: &dyn AttributeStore,
    output: &mut dyn StrokeGeometryBuilder,
) -> Result<(), TessellationError> {
    let requested_join = join.line_join;
    let preparation = if matches!(requested_join, LineJoin::Arcs | LineJoin::ArcsRound) {
        prepare_arcs_join(join, differentials.current(), options)
    } else {
        Ok(PreparedJoin::Standard(requested_join))
    };
    let mut prepared = match preparation {
        Ok(PreparedJoin::Arcs(geometry)) => Some(geometry),
        Ok(PreparedJoin::Standard(style)) => {
            join.line_join = style;
            None
        }
        Err(reason) => {
            join.line_join = arcs.fallback(reason);
            None
        }
    };
    compute_join_positions::<FIXED_WIDTH>(join, neighbors, options, vertex)?;
    if let Some(resolved) = prepared.as_ref() {
        if let Err(reason) = align_arcs_side_points(join, resolved) {
            join.line_join = arcs.fallback(reason);
            prepared = None;
        }
    }
    emit_join_base::<FIXED_WIDTH>(join, neighbors[0], emit_edge, vertex, attributes, output)?;
    tessellate_join::<true>(
        join,
        neighbors,
        requested_join,
        prepared.as_ref(),
        options,
        arcs,
        vertex,
        attributes,
        output,
    )
}

#[inline(always)]
fn compute_join_positions<const FIXED_WIDTH: bool>(
    join: &mut EndpointData,
    [prev, next]: [&EndpointData; 2],
    options: &StrokeOptions,
    vertex: &mut StrokeVertexData,
) -> Result<(), TessellationError> {
    if FIXED_WIDTH {
        compute_join_side_positions_fixed_width(prev, join, next, options.miter_limit, vertex)?;
    } else {
        compute_join_side_positions(prev, join, next, options.miter_limit, SIDE_POSITIVE);
        compute_join_side_positions(prev, join, next, options.miter_limit, SIDE_NEGATIVE);
    }
    Ok(())
}

#[inline(always)]
fn emit_join_base<const FIXED_WIDTH: bool>(
    join: &mut EndpointData,
    prev: &EndpointData,
    emit_edge: bool,
    vertex: &mut StrokeVertexData,
    attributes: &dyn AttributeStore,
    output: &mut dyn StrokeGeometryBuilder,
) -> Result<(), TessellationError> {
    if !FIXED_WIDTH {
        // Prevent folding when the other side is concave, after Arcs alignment.
        if join.side_points[SIDE_POSITIVE].single_vertex.is_some() {
            join.fold[SIDE_NEGATIVE] = false;
        }
        if join.side_points[SIDE_NEGATIVE].single_vertex.is_some() {
            join.fold[SIDE_POSITIVE] = false;
        }
    }
    add_join_base_vertices(join, vertex, attributes, output, Side::Negative)?;
    add_join_base_vertices(join, vertex, attributes, output, Side::Positive)?;
    if emit_edge {
        add_edge_triangles(prev, join, output);
    }
    Ok(())
}

fn resolve_arcs_join(
    join: &EndpointData,
    differentials: EndpointDifferentials,
    options: &StrokeOptions,
) -> Result<JoinConstruction, ArcsJoinFallback> {
    let (
        EndpointDifferential::Regular {
            unit_tangent: incoming_tangent,
            curvature: incoming_curvature,
        },
        EndpointDifferential::Regular {
            unit_tangent: outgoing_tangent,
            curvature: outgoing_curvature,
        },
    ) = (differentials.incoming, differentials.outgoing)
    else {
        return Err(ArcsJoinFallback::MissingDifferentials);
    };

    let input = JoinInput {
        at: Point64::new(f64::from(join.position.x), f64::from(join.position.y)),
        incoming: SegmentEnd {
            tangent: Vector64::new(f64::from(incoming_tangent.x), f64::from(incoming_tangent.y)),
            curvature: incoming_curvature,
        },
        outgoing: SegmentEnd {
            tangent: Vector64::new(f64::from(outgoing_tangent.x), f64::from(outgoing_tangent.y)),
            curvature: outgoing_curvature,
        },
        half_width: f64::from(join.half_width),
        miter_limit: f64::from(options.miter_limit),
    };

    stroke_arcs::construct_svg2(input).map_err(ArcsJoinFallback::Construction)
}

/// Resolve geometry without mutating the endpoint or emitting any vertices.
fn prepare_arcs_join(
    join: &EndpointData,
    differentials: EndpointDifferentials,
    options: &StrokeOptions,
) -> Result<PreparedJoin, ArcsJoinFallback> {
    let geometry = match resolve_arcs_join(join, differentials, options)? {
        JoinConstruction::Empty => return Ok(PreparedJoin::Standard(LineJoin::Miter)),
        JoinConstruction::MiterClip => return Ok(PreparedJoin::Standard(LineJoin::MiterClip)),
        JoinConstruction::ParallelRectangle(rectangle) => {
            PreparedArcsJoin::ParallelRectangle(prepare_parallel_rectangle(rectangle)?)
        }
        JoinConstruction::RadialClip(clip) => {
            PreparedArcsJoin::RadialClip(prepare_radial_clip(clip)?)
        }
        JoinConstruction::Arcs(resolved) => PreparedArcsJoin::Curved(resolved),
    };
    Ok(PreparedJoin::Arcs(geometry))
}

fn prepare_radial_clip(
    radial_clip: RadialClipJoin,
) -> Result<PreparedRadialClip, ArcsJoinFallback> {
    Ok(PreparedRadialClip {
        turn: radial_clip.turn,
        incoming_offset_point: point64_to_point(radial_clip.incoming_offset_point)?,
        outgoing_offset_point: point64_to_point(radial_clip.outgoing_offset_point)?,
        incoming: point64_to_point(radial_clip.incoming)?,
        outgoing: point64_to_point(radial_clip.outgoing)?,
    })
}

fn prepare_parallel_rectangle(
    rectangle: ParallelRectangleJoin,
) -> Result<PreparedParallelRectangle, ArcsJoinFallback> {
    Ok(PreparedParallelRectangle {
        near_left: point64_to_point(rectangle.near_left)?,
        near_right: point64_to_point(rectangle.near_right)?,
        far_left: point64_to_point(rectangle.far_left)?,
        far_right: point64_to_point(rectangle.far_right)?,
    })
}

fn align_arcs_side_points(
    join: &mut EndpointData,
    prepared: &PreparedArcsJoin,
) -> Result<(), ArcsJoinFallback> {
    match prepared {
        PreparedArcsJoin::Curved(resolved) => {
            let incoming = point64_to_point(resolved.incoming_offset_point())?;
            let outgoing = point64_to_point(resolved.outgoing_offset_point())?;
            let side = arcs_outer_side(resolved.turn());
            join.side_points[side].prev = incoming;
            join.side_points[side].next = outgoing;
            join.side_points[side].single_vertex = None;
        }
        PreparedArcsJoin::ParallelRectangle(rectangle) => {
            join.side_points[SIDE_POSITIVE].prev = rectangle.near_left;
            join.side_points[SIDE_POSITIVE].next = rectangle.near_right;
            join.side_points[SIDE_POSITIVE].single_vertex = None;
            join.side_points[SIDE_NEGATIVE].prev = rectangle.near_right;
            join.side_points[SIDE_NEGATIVE].next = rectangle.near_left;
            join.side_points[SIDE_NEGATIVE].single_vertex = None;
            join.fold = [false, false];
        }
        PreparedArcsJoin::RadialClip(radial_clip) => {
            let side = arcs_outer_side(radial_clip.turn);
            join.side_points[side].prev = radial_clip.incoming_offset_point;
            join.side_points[side].next = radial_clip.outgoing_offset_point;
            join.side_points[side].single_vertex = None;
        }
    }
    Ok(())
}

fn arcs_outer_side(turn: TurnSide) -> usize {
    match turn {
        TurnSide::Left => SIDE_NEGATIVE,
        TurnSide::Right => SIDE_POSITIVE,
    }
}

fn point64_to_point(value: Point64) -> Result<Point, ArcsJoinFallback> {
    let minimum = f64::from(f32::MIN);
    let maximum = f64::from(f32::MAX);
    if !value.x.is_finite()
        || !value.y.is_finite()
        || value.x < minimum
        || value.x > maximum
        || value.y < minimum
        || value.y > maximum
    {
        return Err(ArcsJoinFallback::UnrepresentablePoint);
    }

    Ok(point(value.x as f32, value.y as f32))
}

fn tessellate_join<const EXTENDED: bool>(
    join: &mut EndpointData,
    neighbors: [&EndpointData; 2],
    requested_join: LineJoin,
    arcs_join: Option<&PreparedArcsJoin>,
    options: &StrokeOptions,
    workspace: &mut JoinWorkspace,
    vertex: &mut StrokeVertexData,
    attributes: &dyn AttributeStore,
    output: &mut (impl StrokeGeometryBuilder + ?Sized),
) -> Result<(), TessellationError> {
    debug_assert!(
        !matches!(join.line_join, LineJoin::Arcs | LineJoin::ArcsRound) || arcs_join.is_some()
    );

    if EXTENDED {
        if let Some(PreparedArcsJoin::ParallelRectangle(rectangle)) = arcs_join {
            emit_parallel_arcs_rectangle(join, rectangle, vertex, attributes, output)?;
            if requested_join == LineJoin::ArcsRound && options.miter_limit > 0.0 {
                round_clip::emit(
                    join,
                    [rectangle.far_left, rectangle.far_right],
                    [
                        rectangle.far_left - rectangle.near_left,
                        rectangle.far_right - rectangle.near_right,
                    ]
                    .map(round_clip::vector64),
                    SIDE_POSITIVE,
                    options.tolerance,
                    vertex,
                    attributes,
                    output,
                )?;
            }
            return Ok(());
        }

        if let Some(PreparedArcsJoin::RadialClip(radial_clip)) = arcs_join {
            if emit_clipped_join(
                join,
                [radial_clip.incoming, radial_clip.outgoing],
                arcs_outer_side(radial_clip.turn),
                vertex,
                attributes,
                output,
            )? {
                if requested_join == LineJoin::ArcsRound {
                    round_clip::emit(
                        join,
                        [radial_clip.incoming, radial_clip.outgoing],
                        [
                            radial_clip.incoming - join.position,
                            radial_clip.outgoing - join.position,
                        ]
                        .map(round_clip::vector64),
                        arcs_outer_side(radial_clip.turn),
                        options.tolerance,
                        vertex,
                        attributes,
                        output,
                    )?;
                }
                return Ok(());
            }
        }

        if let Some(PreparedArcsJoin::Curved(resolved)) = arcs_join {
            if resolved.clips_radial_edge()
                && emit_mixed_arcs_clip(
                    join,
                    resolved,
                    options,
                    &mut workspace.mesh,
                    &mut workspace.vertex_ids,
                    vertex,
                    attributes,
                    output,
                )?
                .emitted(workspace)
            {
                if requested_join == LineJoin::ArcsRound {
                    if let Some([a, b]) = resolved.clip_endpoints() {
                        if let [Ok(a), Ok(b)] = [point64_to_point(a), point64_to_point(b)] {
                            round_clip::emit(
                                join,
                                [a, b],
                                resolved.clip_tangents(),
                                arcs_outer_side(resolved.turn()),
                                options.tolerance,
                                vertex,
                                attributes,
                                output,
                            )?;
                        }
                    }
                }
                return Ok(());
            }
        }

        // A subunit cut can cross the radial bevel edges. Keep the segment
        // attachments intact and emit the retained join separately instead of
        // extending the clipping line backwards into the segment bodies.
        if join.line_join == LineJoin::MiterClip && options.miter_limit < 1.0 {
            for side in 0..2 {
                if join.side_points[side].single_vertex.is_some() {
                    continue;
                }
                if let Some((ends, tangents)) = subunit_miter_clip(join, side, options.miter_limit)
                {
                    if emit_clipped_join(join, ends, side, vertex, attributes, output)? {
                        if requested_join == LineJoin::ArcsRound {
                            round_clip::emit(
                                join,
                                ends,
                                tangents,
                                side,
                                options.tolerance,
                                vertex,
                                attributes,
                                output,
                            )?;
                        }
                        return Ok(());
                    }
                }
            }
        }
    }

    let side_needs_join = [
        join.side_points[SIDE_POSITIVE].single_vertex.is_none() && !join.fold[SIDE_NEGATIVE],
        join.side_points[SIDE_NEGATIVE].single_vertex.is_none() && !join.fold[SIDE_POSITIVE],
    ];

    if !join.fold[SIDE_POSITIVE] && !join.fold[SIDE_NEGATIVE] {
        // Tessellate the interior of the join.
        match side_needs_join {
            [true, true] => {
                output.add_triangle(
                    join.side_points[SIDE_POSITIVE].prev_vertex,
                    join.side_points[SIDE_POSITIVE].next_vertex,
                    join.side_points[SIDE_NEGATIVE].next_vertex,
                );

                output.add_triangle(
                    join.side_points[SIDE_POSITIVE].prev_vertex,
                    join.side_points[SIDE_NEGATIVE].next_vertex,
                    join.side_points[SIDE_NEGATIVE].prev_vertex,
                );
            }
            [false, true] => {
                output.add_triangle(
                    join.side_points[SIDE_NEGATIVE].prev_vertex,
                    join.side_points[SIDE_POSITIVE].prev_vertex,
                    join.side_points[SIDE_NEGATIVE].next_vertex,
                );
            }
            [true, false] => {
                output.add_triangle(
                    join.side_points[SIDE_NEGATIVE].prev_vertex,
                    join.side_points[SIDE_POSITIVE].prev_vertex,
                    join.side_points[SIDE_POSITIVE].next_vertex,
                );
            }
            [false, false] => {}
        }
    }

    // Tessellate the remaining specific shape for convex joins
    for side in 0..2 {
        if !side_needs_join[side] {
            continue;
        }

        match join.line_join {
            LineJoin::Round => {
                tessellate_round_join(join, side, options, vertex, attributes, output)?;
            }
            LineJoin::Arcs | LineJoin::ArcsRound if EXTENDED => {
                let emitted = if let Some(PreparedArcsJoin::Curved(resolved)) = arcs_join {
                    side == arcs_outer_side(resolved.turn())
                        && emit_arcs_join(
                            join,
                            resolved,
                            side,
                            options,
                            &mut workspace.mesh,
                            &mut workspace.vertex_ids,
                            vertex,
                            attributes,
                            output,
                        )?
                        .emitted(workspace)
                } else {
                    false
                };
                if emitted && requested_join == LineJoin::ArcsRound {
                    if let Some(PreparedArcsJoin::Curved(resolved)) = arcs_join {
                        if let Some(ends) = resolved.clip_endpoints() {
                            let ends = [point64_to_point(ends[0]), point64_to_point(ends[1])];
                            if let [Ok(a), Ok(b)] = ends {
                                round_clip::emit(
                                    join,
                                    [a, b],
                                    resolved.clip_tangents(),
                                    side,
                                    options.tolerance,
                                    vertex,
                                    attributes,
                                    output,
                                )?;
                            }
                        }
                    }
                }
                if !emitted {
                    tessellate_round_join(join, side, options, vertex, attributes, output)?;
                }
            }
            LineJoin::MiterClip if EXTENDED && requested_join == LineJoin::ArcsRound => {
                // A surviving pair of distinct outer vertices is the actual
                // miter cut. Unclipped miters have a single shared vertex.
                round_clip::emit(
                    join,
                    [join.side_points[side].prev, join.side_points[side].next],
                    [
                        round_clip::edge_tangent(neighbors[0], join, side),
                        -round_clip::edge_tangent(join, neighbors[1], side),
                    ],
                    side,
                    options.tolerance,
                    vertex,
                    attributes,
                    output,
                )?;
            }
            _ => {}
        }
    }

    Ok(())
}

fn emit_parallel_arcs_rectangle(
    join: &EndpointData,
    rectangle: &PreparedParallelRectangle,
    vertex: &mut StrokeVertexData,
    attributes: &dyn AttributeStore,
    output: &mut (impl StrokeGeometryBuilder + ?Sized),
) -> Result<(), TessellationError> {
    if rectangle.far_left == join.side_points[SIDE_POSITIVE].prev
        && rectangle.far_right == join.side_points[SIDE_NEGATIVE].prev
    {
        return Ok(());
    }

    vertex.position_on_path = join.position;
    vertex.half_width = join.half_width;
    vertex.side = Side::Positive;
    vertex.normal = (rectangle.far_left - join.position) / join.half_width;
    let far_left_vertex = output.add_stroke_vertex(StrokeVertex(vertex, attributes))?;
    vertex.side = Side::Negative;
    vertex.normal = (rectangle.far_right - join.position) / join.half_width;
    let far_right_vertex = output.add_stroke_vertex(StrokeVertex(vertex, attributes))?;

    let near_left_vertex = join.side_points[SIDE_POSITIVE].prev_vertex;
    let near_right_vertex = join.side_points[SIDE_NEGATIVE].prev_vertex;
    output.add_triangle(near_left_vertex, far_left_vertex, far_right_vertex);
    output.add_triangle(near_left_vertex, far_right_vertex, near_right_vertex);
    Ok(())
}

/// Fill between unchanged segment attachments and a pair of clipped join edges.
fn emit_clipped_join(
    join: &EndpointData,
    [incoming, outgoing]: [Point; 2],
    outer_side: usize,
    vertex: &mut StrokeVertexData,
    attributes: &dyn AttributeStore,
    output: &mut (impl StrokeGeometryBuilder + ?Sized),
) -> Result<bool, TessellationError> {
    let inner_side = 1 - outer_side;
    if join.side_points[inner_side].single_vertex.is_none()
        || join.fold[outer_side]
        || join.fold[inner_side]
    {
        return Ok(false);
    }

    vertex.position_on_path = join.position;
    vertex.half_width = join.half_width;
    vertex.side = if outer_side == SIDE_POSITIVE {
        Side::Positive
    } else {
        Side::Negative
    };

    vertex.normal = (incoming - join.position) / join.half_width;
    let incoming_clip_vertex = output.add_stroke_vertex(StrokeVertex(vertex, attributes))?;
    let outgoing_clip_vertex = if outgoing == incoming {
        incoming_clip_vertex
    } else {
        vertex.normal = (outgoing - join.position) / join.half_width;
        output.add_stroke_vertex(StrokeVertex(vertex, attributes))?
    };

    let inner_vertex = join.side_points[inner_side].prev_vertex;
    let incoming_outer_vertex = join.side_points[outer_side].prev_vertex;
    let outgoing_outer_vertex = join.side_points[outer_side].next_vertex;
    match outer_side {
        SIDE_NEGATIVE => {
            output.add_triangle(inner_vertex, outgoing_outer_vertex, outgoing_clip_vertex);
            if outgoing_clip_vertex != incoming_clip_vertex {
                output.add_triangle(inner_vertex, outgoing_clip_vertex, incoming_clip_vertex);
            }
            output.add_triangle(inner_vertex, incoming_clip_vertex, incoming_outer_vertex);
        }
        _ => {
            output.add_triangle(inner_vertex, incoming_outer_vertex, incoming_clip_vertex);
            if incoming_clip_vertex != outgoing_clip_vertex {
                output.add_triangle(inner_vertex, incoming_clip_vertex, outgoing_clip_vertex);
            }
            output.add_triangle(inner_vertex, outgoing_clip_vertex, outgoing_outer_vertex);
        }
    }

    Ok(true)
}

/// Find each cut on its radial edge or its straight support, as appropriate.
fn subunit_miter_clip(
    join: &EndpointData,
    side: usize,
    limit: f32,
) -> Option<([Point; 2], [Vector64; 2])> {
    let offsets = [
        join.side_points[side].prev - join.position,
        join.side_points[side].next - join.position,
    ];
    let bisector = (offsets[0] + offsets[1]).try_normalize()?;
    let distance = limit * join.half_width;
    let (incoming, outgoing) = get_clip_intersections(offsets[0], offsets[1], bisector, distance);
    let mut cuts = [incoming, outgoing];
    let mut tangents = [Vector64::new(0.0, 0.0); 2];
    for i in 0..2 {
        let projection = offsets[i].dot(bisector);
        if projection > distance {
            cuts[i] = offsets[i] * (distance / projection);
            tangents[i] = round_clip::vector64(offsets[i]);
        } else {
            tangents[i] = round_clip::vector64(cuts[i] - offsets[i]);
        }
    }
    Some((cuts.map(|offset| join.position + offset), tangents))
}

fn set_join_vertex_metadata(join: &EndpointData, side: usize, vertex: &mut StrokeVertexData) {
    vertex.position_on_path = join.position;
    vertex.half_width = join.half_width;
    vertex.side = if side == SIDE_POSITIVE {
        Side::Positive
    } else {
        Side::Negative
    };
}

#[allow(clippy::too_many_arguments)]
fn emit_mixed_arcs_clip(
    join: &EndpointData,
    resolved: &ResolvedArcsJoin,
    options: &StrokeOptions,
    mesh: &mut ArcsMesh,
    vertex_ids: &mut Vec<VertexId>,
    vertex: &mut StrokeVertexData,
    attributes: &dyn AttributeStore,
    output: &mut (impl StrokeGeometryBuilder + ?Sized),
) -> Result<JoinEmission, TessellationError> {
    let side = arcs_outer_side(resolved.turn());
    let inner_side = 1 - side;
    let Some(inner_position) = join.side_points[inner_side].single_vertex else {
        return Ok(JoinEmission::Fallback(
            ArcsJoinFallback::IncompatibleAttachments,
        ));
    };
    if join.fold[side] || join.fold[inner_side] {
        return Ok(JoinEmission::Fallback(
            ArcsJoinFallback::IncompatibleAttachments,
        ));
    }
    if let Err(error) = mesh.tessellate_with_inner_vertex(
        resolved,
        f64::from(options.tolerance),
        Point64::new(f64::from(inner_position.x), f64::from(inner_position.y)),
    ) {
        return Ok(JoinEmission::Fallback(ArcsJoinFallback::Mesh(error)));
    }

    // The contour ends with the existing outgoing attachment and inner vertex.
    // Validate before emitting anything so a failed conversion can fall back.
    let vertices = mesh.vertices();
    let middle = &vertices[1..vertices.len() - 2];
    for p in middle {
        if let Err(reason) = point64_to_point(*p) {
            return Ok(JoinEmission::Fallback(reason));
        }
    }
    vertex_ids.clear();
    vertex_ids.reserve(vertices.len());
    vertex_ids.push(join.side_points[side].prev_vertex);
    set_join_vertex_metadata(join, side, vertex);
    for p in middle {
        vertex.normal = (point(p.x as f32, p.y as f32) - join.position) / join.half_width;
        vertex_ids.push(output.add_stroke_vertex(StrokeVertex(vertex, attributes))?);
    }
    vertex_ids.push(join.side_points[side].next_vertex);
    vertex_ids.push(join.side_points[inner_side].prev_vertex);
    for triangle in mesh.indices().chunks_exact(3) {
        output.add_triangle(
            vertex_ids[triangle[0]],
            vertex_ids[triangle[2]],
            vertex_ids[triangle[1]],
        );
    }
    Ok(JoinEmission::Emitted)
}

#[allow(clippy::too_many_arguments)]
fn emit_arcs_join(
    join: &EndpointData,
    resolved: &ResolvedArcsJoin,
    side: usize,
    options: &StrokeOptions,
    mesh: &mut ArcsMesh,
    vertex_ids: &mut Vec<VertexId>,
    vertex: &mut StrokeVertexData,
    attributes: &dyn AttributeStore,
    output: &mut (impl StrokeGeometryBuilder + ?Sized),
) -> Result<JoinEmission, TessellationError> {
    if resolved.clips_radial_edge() {
        // Mixed clips need the full inner contour, handled before bevel fill.
        return Ok(JoinEmission::Fallback(
            ArcsJoinFallback::IncompatibleAttachments,
        ));
    }
    let tessellation = match mesh.tessellate_for_output(resolved, f64::from(options.tolerance)) {
        Ok(tessellation) => tessellation,
        Err(error) => return Ok(JoinEmission::Fallback(ArcsJoinFallback::Mesh(error))),
    };

    match tessellation {
        ArcsOutput::Triangle(triangle) => {
            let Ok(position) = point64_to_point(triangle.middle) else {
                return Ok(JoinEmission::Fallback(
                    ArcsJoinFallback::UnrepresentablePoint,
                ));
            };
            if position == join.position {
                return Ok(JoinEmission::Fallback(ArcsJoinFallback::CollapsedVertex));
            }

            set_join_vertex_metadata(join, side, vertex);
            vertex.normal = (position - join.position) / join.half_width;
            let middle_vertex = output.add_stroke_vertex(StrokeVertex(vertex, attributes))?;
            let vertex_ids = [
                join.side_points[side].prev_vertex,
                middle_vertex,
                join.side_points[side].next_vertex,
            ];
            output.add_triangle(
                vertex_ids[triangle.indices[0]],
                vertex_ids[triangle.indices[2]],
                vertex_ids[triangle.indices[1]],
            );
            Ok(JoinEmission::Emitted)
        }
        ArcsOutput::Quad(quad) => {
            let Ok(first_position) = point64_to_point(quad.middle[0]) else {
                return Ok(JoinEmission::Fallback(
                    ArcsJoinFallback::UnrepresentablePoint,
                ));
            };
            let Ok(second_position) = point64_to_point(quad.middle[1]) else {
                return Ok(JoinEmission::Fallback(
                    ArcsJoinFallback::UnrepresentablePoint,
                ));
            };
            if first_position == join.position || second_position == join.position {
                return Ok(JoinEmission::Fallback(ArcsJoinFallback::CollapsedVertex));
            }

            set_join_vertex_metadata(join, side, vertex);
            vertex.normal = (first_position - join.position) / join.half_width;
            let first_middle = output.add_stroke_vertex(StrokeVertex(vertex, attributes))?;
            vertex.normal = (second_position - join.position) / join.half_width;
            let second_middle = output.add_stroke_vertex(StrokeVertex(vertex, attributes))?;
            let vertex_ids = [
                join.side_points[side].prev_vertex,
                first_middle,
                second_middle,
                join.side_points[side].next_vertex,
            ];
            for triangle in quad.indices.chunks_exact(3) {
                output.add_triangle(
                    vertex_ids[usize::from(triangle[0])],
                    vertex_ids[usize::from(triangle[2])],
                    vertex_ids[usize::from(triangle[1])],
                );
            }
            Ok(JoinEmission::Emitted)
        }
        ArcsOutput::Fan { vertices, fan } => emit_arcs_mesh_output(
            join,
            side,
            vertices,
            MeshTriangles::Fan(fan),
            vertex_ids,
            vertex,
            attributes,
            output,
        ),
        ArcsOutput::Mesh { vertices, indices } => emit_arcs_mesh_output(
            join,
            side,
            vertices,
            MeshTriangles::Indexed(indices),
            vertex_ids,
            vertex,
            attributes,
            output,
        ),
    }
}

enum MeshTriangles<'a> {
    Fan(ValidatedFan),
    Indexed(&'a [usize]),
}

#[allow(clippy::too_many_arguments)]
fn emit_arcs_mesh_output(
    join: &EndpointData,
    side: usize,
    vertices: &[Point64],
    triangles: MeshTriangles,
    vertex_ids: &mut Vec<VertexId>,
    vertex: &mut StrokeVertexData,
    attributes: &dyn AttributeStore,
    output: &mut (impl StrokeGeometryBuilder + ?Sized),
) -> Result<JoinEmission, TessellationError> {
    if vertices.len() < 3 {
        return Ok(JoinEmission::Fallback(
            ArcsJoinFallback::IncompatibleAttachments,
        ));
    }

    for point64 in &vertices[1..vertices.len() - 1] {
        let Ok(position) = point64_to_point(*point64) else {
            return Ok(JoinEmission::Fallback(
                ArcsJoinFallback::UnrepresentablePoint,
            ));
        };
        if position == join.position {
            return Ok(JoinEmission::Fallback(ArcsJoinFallback::CollapsedVertex));
        }
    }

    vertex_ids.clear();
    vertex_ids.reserve(vertices.len());
    vertex_ids.push(join.side_points[side].prev_vertex);

    set_join_vertex_metadata(join, side, vertex);
    for point64 in &vertices[1..vertices.len() - 1] {
        // The preflight above checked both components against the f32 range.
        let position = point(point64.x as f32, point64.y as f32);
        vertex.normal = (position - join.position) / join.half_width;
        vertex_ids.push(output.add_stroke_vertex(StrokeVertex(vertex, attributes))?);
    }
    vertex_ids.push(join.side_points[side].next_vertex);

    if let MeshTriangles::Fan(fan) = triangles {
        for [first, second, third] in fan.triangles() {
            output.add_triangle(vertex_ids[first], vertex_ids[third], vertex_ids[second]);
        }
    } else if let MeshTriangles::Indexed(indices) = triangles {
        for triangle in indices.chunks_exact(3) {
            output.add_triangle(
                vertex_ids[triangle[0]],
                vertex_ids[triangle[2]],
                vertex_ids[triangle[1]],
            );
        }
    }

    Ok(JoinEmission::Emitted)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn public_pipeline_records_degenerate_fallback_without_masking_valid_arcs() {
        use crate::geometry_builder::{simple_builder, VertexBuffers};
        for style in [LineJoin::Arcs, LineJoin::ArcsRound] {
            for degenerate in [false, true] {
                let mut path = crate::path::Path::builder();
                path.begin(point(-40.0, 0.0));
                path.line_to(point(0.0, 0.0));
                let ctrl = if degenerate {
                    point(0.0, 0.0)
                } else {
                    point(0.0, 10.0)
                };
                path.quadratic_bezier_to(ctrl, point(-20.0, 40.0));
                path.end(false);
                let mut tess = StrokeTessellator::new();
                let mut mesh: VertexBuffers<Point, u16> = VertexBuffers::new();
                tess.tessellate_path(
                    &path.build(),
                    &StrokeOptions::default()
                        .with_line_join(style)
                        .with_line_width(2.0),
                    &mut simple_builder(&mut mesh),
                )
                .unwrap();
                if degenerate {
                    assert_eq!(
                        tess.arcs.joins.last_fallback,
                        Some(ArcsJoinFallback::MissingDifferentials)
                    );
                } else {
                    assert_eq!(tess.arcs.joins.last_fallback, None);
                }
                assert!(!mesh.indices.is_empty());
            }
        }
    }

    #[test]
    fn construction_failures_keep_the_solver_error() {
        let join = EndpointData {
            half_width: 0.0,
            ..EndpointData::default()
        };
        let differentials = EndpointDifferentials {
            incoming: EndpointDifferential::Regular {
                unit_tangent: vector(1.0, 0.0),
                curvature: 0.1,
            },
            outgoing: EndpointDifferential::Regular {
                unit_tangent: vector(0.0, 1.0),
                curvature: 0.1,
            },
        };
        assert!(matches!(
            prepare_arcs_join(&join, differentials, &StrokeOptions::default()),
            Err(ArcsJoinFallback::Construction(_))
        ));
        assert!(matches!(
            point64_to_point(Point64::new(f64::INFINITY, 0.0)),
            Err(ArcsJoinFallback::UnrepresentablePoint)
        ));
    }

    #[test]
    fn arcs_join_resolves_endpoint_differentials_into_support_geometry() {
        let join = EndpointData {
            position: point(10.0, 5.0),
            half_width: 2.0,
            line_join: LineJoin::Arcs,
            ..EndpointData::default()
        };
        let differentials = EndpointDifferentials {
            incoming: EndpointDifferential::Regular {
                unit_tangent: vector(1.0, 0.0),
                curvature: 0.0,
            },
            outgoing: EndpointDifferential::Regular {
                unit_tangent: vector(0.0, 1.0),
                curvature: 0.1,
            },
        };

        let resolution = resolve_arcs_join(&join, differentials, &StrokeOptions::default());

        assert!(matches!(resolution, Ok(JoinConstruction::Arcs(_))));
    }
    #[test]
    fn arcs_join_keeps_explicit_svg_fallback_states() {
        let join = EndpointData {
            half_width: 2.0,
            line_join: LineJoin::Arcs,
            ..EndpointData::default()
        };
        let line_differentials = EndpointDifferentials {
            incoming: EndpointDifferential::Regular {
                unit_tangent: vector(1.0, 0.0),
                curvature: 0.0,
            },
            outgoing: EndpointDifferential::Regular {
                unit_tangent: vector(0.0, 1.0),
                curvature: 0.0,
            },
        };

        let options = StrokeOptions::default();
        let line_join = resolve_arcs_join(&join, line_differentials, &options);
        let line_geometry = prepare_arcs_join(&join, line_differentials, &options);
        let degenerate_differentials = EndpointDifferentials {
            outgoing: EndpointDifferential::Degenerate,
            ..line_differentials
        };
        let degenerate_join = resolve_arcs_join(&join, degenerate_differentials, &options);
        let degenerate_geometry = prepare_arcs_join(&join, degenerate_differentials, &options);

        assert!(matches!(line_join, Ok(JoinConstruction::MiterClip)));
        assert!(matches!(
            line_geometry,
            Ok(PreparedJoin::Standard(LineJoin::MiterClip))
        ));
        assert!(degenerate_join.is_err());
        assert!(matches!(
            degenerate_geometry,
            Err(ArcsJoinFallback::MissingDifferentials)
        ));
        assert_eq!(join.line_join, LineJoin::Arcs);
    }
}
