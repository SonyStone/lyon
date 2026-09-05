# Arcs joins

`LineJoin::Arcs` constructs a join from the original Bezier endpoint tangents
and curvature. `LineJoin::ArcsRound` adds tangent-continuous rounding to actual
miter cuts. Both use the existing stroke entry points and vertex metadata.

## Compatibility and release requirements

Adding variants to the exhaustive `lyon_path::LineJoin` enum is a source-breaking
change. Downstream exhaustive matches must handle both new variants. Preserving
the existing Serde names and indices does not make the source API compatible.
This change must be included in a release that permits breaking changes; it is
not a drop-in compatible 1.x update. Package versions are still the repository's
development versions and need coordinated release/version selection before
publication.

`StrokeOptions` has no additional fields. Existing entry points retain their
output-builder dispatch. The default join remains `Miter`. The miter-limit setter
now accepts zero and subunit values for every style, independently of setter
order. `MINIMUM_MITER_LIMIT` remains the legacy value of one for existing callers.

ArcsRound is a Lyon extension. Its tip can extend beyond the miter limit, and
invalid or non-convex biarc fits retain the flat cut. Line caps are independent.
Pre-flattened paths have no original curvature; elliptical arcs converted to
Beziers use the approximation's endpoint curvature. Variable-width supports use
the width at the join without derivatives of the width profile.

## Internal responsibilities

- `stroke_history.rs` owns the endpoint history and its optional derivatives.
  Push, replacement, saving the first endpoints and closing a contour update
  both histories together. It keeps derivative storage separate from the compact
  `EndpointData`. Only the Arcs endpoint adapter queries the derivative history.
- `stroke_join.rs` owns endpoint preparation, effective join selection,
  attachment alignment, local mesh-to-vertex mapping, winding and fallback
  policy. The path walker does not inspect Triangle/Quad/Fan/Mesh output.
- `stroke_arcs.rs` and `stroke_arcs/svg2.rs` implement the geometric construction
  without access to the output builder or attributes.
- `stroke_arcs_mesh.rs` flattens and triangulates the resolved boundary, reusing
  buffers. Specialized triangle, quad and fan output stays available. Buffered
  reference entry points are test-only.

Preparation returns `PreparedJoin` or a typed fallback cause and does not mutate
the endpoint. The adapter applies that result and owns the emission order.
Missing derivatives, construction errors, unrepresentable positions and mesh
failures remain distinguishable until the adapter selects Round. Test builds
retain the last fallback cause to exercise this policy through the public stroke
pipeline. Output-builder errors always abort tessellation and are not geometric
fallbacks.

For mixed clips, the retained polygon must be emitted before the ordinary bevel
fill. The adapter therefore handles them before the shared join interior. Local
mesh indices referring to incoming, outgoing and inner attachment vertices never
escape into the path walker. Existing vertex insertion order and metadata are
preserved by this refactoring.

## Why disabled Arcs can affect performance

A runtime branch avoids executing the arc solver, but does not undo changes to
the containing function's compiled stack frame, register spills or inlining.
Keeping Arcs preparation in the step loop enlarged the stack frame even for
ordinary joins. History wrappers can also add calls and copies on every flattened
point despite derivative tracking being disabled. In the measured Windows build,
forwarding the 112-byte `EndpointData` by value added copies even with forced
inlining; borrowing the payload avoids that intermediate copy.

The extended endpoint path is deliberately outlined. Ordinary joins instantiate
the shared emitter without extended branches. Flattened points bypass endpoint
preparation, and the small history push/replacement operations take borrowed
payloads and are inlined.
These choices preserve module ownership without requiring dynamic interfaces or
heap allocation per join. Do not infer a speedup solely from reduced stack size;
compare the generated code and benchmarks after changing these boundaries.

### Measured legacy-join cost

On Windows x86_64, Intel Core i9-13900H, Rust 1.90.0, default release settings,
three sequential series pinned to the same logical CPU produced these median
nanoseconds per iteration. The order was base/before/after, after/before/base,
then before/base/after; all builds completed before measurements. Base is
`994526f`, before is `2ab9cdf`, and after includes this architectural refactoring.

| Logo benchmark | Base | Before | After | After vs before |
| --- | ---: | ---: | ---: | ---: |
| Miter | 46,313 | 45,425 | 43,728 | -3.7% |
| Bevel | 47,971 | 49,467 | 45,663 | -7.7% |
| Round | 59,465 | 63,522 | 59,172 | -6.8% |

This series did not reproduce the earlier regression against base. The spread
on this interactive machine is substantial: base Round ranged from 57,216 to
61,430 ns/iteration, for example. These results support the boundary changes;
they do not establish statistical equivalence or performance on other targets.

Three alternating before/after pairs of the `arcs_` benchmarks, using the same
builds and CPU affinity, also reduced the measured medians: curved Arcs
242,718 to 224,212 ns, clipped Arcs 215,261 to 197,788 ns, straight-segment Arcs
24,761 to 22,642 ns, and ArcsRound 238,237 to 225,463 ns. These workloads reuse
the tessellator and output buffers; the same measurement limitations apply.

## Validation

Existing integration tests exercise fixed and variable widths, attributes,
closed paths, style changes and tessellator reuse. Additional tests cover
history wraparound, replaced flattening points, explicit and implicit closure,
disabled derivative evaluation, typed fallback selection and failures at every
output vertex insertion.

The unchanged legacy benchmarks are:

```sh
cargo run -p tess_bench --release -- stroke_
```

Same-input Arcs comparisons with reused tessellator/output buffers are:

```sh
cargo run -p tess_bench --release -- arcs_
```

Run baseline and candidate sequentially with the same toolchain and release
settings, alternating order. Complete builds before measurements. Small timing
differences on an interactive machine need repeated measurements; they are not
evidence of cross-platform performance parity. SVG degenerate-handle conformance,
width-profile derivatives and the existing stroke-overlap limitations remain
separate from this internal refactoring.
