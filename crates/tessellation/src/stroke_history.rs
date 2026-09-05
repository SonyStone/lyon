//! Owns structural changes to endpoint geometry and its optional derivatives.
//! Endpoint payloads are borrowed: forwarding the 112-byte Copy value through
//! another by-value parameter adds copies on the flattened-point hot path.

use super::{
    line_differentials, EndpointData, EndpointDifferentials, Point, PointBuffer,
    SegmentDifferentials,
};
use crate::geom::arrayvec::ArrayVec;

pub(super) struct EndpointHistory<'a> {
    points: PointBuffer,
    firsts: ArrayVec<EndpointData, 2>,
    differentials: &'a mut DifferentialHistory,
    enabled: bool,
}

impl<'a> EndpointHistory<'a> {
    pub(super) fn new(differentials: &'a mut DifferentialHistory, enabled: bool) -> Self {
        if enabled {
            differentials.reset_differentials(0);
        }
        Self {
            points: PointBuffer::new(),
            firsts: ArrayVec::new(),
            differentials,
            enabled,
        }
    }

    pub(super) fn enable_differentials(&mut self) {
        if !self.enabled {
            self.differentials.reset_differentials(self.points.count());
            for _ in 0..self.firsts.len() {
                self.differentials
                    .firsts
                    .push(EndpointDifferentials::default());
            }
            self.enabled = true;
        }
    }

    pub(super) fn record_segment(&mut self, compute: impl FnOnce() -> SegmentDifferentials) {
        if self.enabled {
            self.differentials.record_segment(compute());
        }
    }

    #[inline(always)]
    pub(super) fn push(&mut self, point: &EndpointData) {
        if self.enabled {
            self.differentials.push_next(point.src.is_endpoint());
        }
        self.points.push(*point);
    }

    #[inline(always)]
    pub(super) fn replace_last(&mut self, point: &EndpointData) {
        if self.enabled {
            self.differentials
                .replace_last_with_next(point.src.is_endpoint());
        }
        self.points.replace_last(*point);
    }

    #[inline]
    pub(super) fn capture_firsts(&mut self) {
        let (prev, join) = self.points.last_two();
        self.firsts.push(*prev);
        self.firsts.push(*join);
        if self.enabled {
            self.differentials.capture_firsts();
        }
    }

    pub(super) fn first_for_close(&mut self) -> &EndpointData {
        if self.enabled {
            self.differentials
                .prepare_first_for_close(self.points.last().position, self.firsts[0].position);
        }
        &self.firsts[0]
    }

    pub(super) fn second_for_close(&mut self) -> Option<&EndpointData> {
        let second = self.firsts.get(1)?;
        if self.enabled {
            self.differentials.prepare_second_for_close();
        }
        Some(second)
    }

    pub(super) fn clear(&mut self) {
        self.points.clear();
        self.firsts.clear();
        if self.enabled {
            self.differentials.clear();
        }
    }

    pub(super) fn join_mut(
        &mut self,
    ) -> (&mut EndpointData, &mut EndpointData, &DifferentialHistory) {
        let (prev, join) = self.points.last_two_mut();
        (prev, join, self.differentials)
    }

    pub(super) fn count(&self) -> usize {
        self.points.count()
    }
    pub(super) fn get(&self, index: usize) -> &EndpointData {
        self.points.get(index)
    }
    pub(super) fn last(&self) -> &EndpointData {
        self.points.last()
    }
    pub(super) fn last_mut(&mut self) -> &mut EndpointData {
        self.points.last_mut()
    }
    pub(super) fn last_two_mut(&mut self) -> (&mut EndpointData, &mut EndpointData) {
        self.points.last_two_mut()
    }
    pub(super) fn firsts(&self) -> &[EndpointData] {
        &self.firsts
    }
}

pub(super) struct DifferentialHistory {
    point_buffer: PointBuffer<EndpointDifferentials>,
    firsts: ArrayVec<EndpointDifferentials, 2>,
    next: EndpointDifferentials,
}

impl Default for DifferentialHistory {
    fn default() -> Self {
        Self {
            point_buffer: PointBuffer::new(),
            firsts: ArrayVec::new(),
            next: EndpointDifferentials::default(),
        }
    }
}

impl DifferentialHistory {
    pub(super) fn current(&self) -> EndpointDifferentials {
        *self.point_buffer.last()
    }

    fn reset_differentials(&mut self, point_count: usize) {
        self.point_buffer.clear();
        self.firsts.clear();
        self.next = EndpointDifferentials::default();
        for _ in 0..point_count {
            self.point_buffer.push(EndpointDifferentials::default());
        }
    }

    fn record_segment(&mut self, differentials: SegmentDifferentials) {
        self.point_buffer.last_mut().outgoing = differentials.start;
        self.next.incoming = differentials.end;
    }

    fn push_next(&mut self, is_endpoint: bool) {
        let next = self.take_next(is_endpoint);
        self.point_buffer.push(next);
    }

    fn replace_last_with_next(&mut self, is_endpoint: bool) {
        let next = self.take_next(is_endpoint);
        self.point_buffer.replace_last(next);
    }

    fn take_next(&mut self, is_endpoint: bool) -> EndpointDifferentials {
        if !is_endpoint {
            return EndpointDifferentials::default();
        }

        let next = self.next;
        self.next = EndpointDifferentials::default();
        next
    }

    fn capture_firsts(&mut self) {
        let (prev, join) = self.point_buffer.last_two();
        self.firsts.push(*prev);
        self.firsts.push(*join);
    }

    fn prepare_first_for_close(&mut self, last_position: Point, first_position: Point) {
        let mut first = self.firsts[0];
        if last_position == first_position {
            self.point_buffer.last_mut().outgoing = first.outgoing;
        } else {
            let closing = line_differentials(last_position, first_position);
            self.point_buffer.last_mut().outgoing = closing.start;
            first.incoming = closing.end;
        }
        self.next = first;
    }

    fn prepare_second_for_close(&mut self) {
        self.next = self.firsts[1];
    }

    fn clear(&mut self) {
        self.reset_differentials(0);
    }
}

#[cfg(test)]
mod tests {
    use super::super::{point, vector, EndpointDifferential, EndpointId, VertexSource};
    use super::*;

    #[test]
    fn ordinary_history_never_computes_derivatives() {
        let mut storage = DifferentialHistory::default();
        let mut history = EndpointHistory::new(&mut storage, false);
        history.push(&endpoint(0.0));
        history.record_segment(|| panic!("derivatives are disabled"));
        history.push(&endpoint(1.0));
        assert_eq!(history.count(), 2);
    }

    #[test]
    fn replaced_flattening_points_keep_pending_endpoint_derivatives() {
        let mut storage = DifferentialHistory::default();
        let mut history = EndpointHistory::new(&mut storage, true);
        history.push(&endpoint(0.0));
        history.record_segment(|| segment(0.25));
        for x in 1..8 {
            let mut sample = endpoint(x as f32);
            sample.src = VertexSource::Edge {
                from: EndpointId(0),
                to: EndpointId(1),
                t: x as f32 / 10.0,
            };
            history.push(&sample);
            history.replace_last(&sample);
            assert_eq!(
                history.join_mut().2.current().incoming,
                EndpointDifferential::None
            );
        }
        history.replace_last(&endpoint(10.0));
        assert_eq!(history.join_mut().2.current().incoming, segment(0.25).end);
    }

    #[test]
    fn enabling_after_wraparound_preserves_implicit_closure() {
        let mut storage = DifferentialHistory::default();
        let mut history = EndpointHistory::new(&mut storage, false);
        history.push(&endpoint(0.0));
        history.push(&endpoint(1.0));
        history.capture_firsts();
        for x in 2..9 {
            history.push(&endpoint(x as f32));
        }
        history.enable_differentials();
        history.record_segment(|| segment(0.5));
        history.push(&endpoint(9.0));
        assert_eq!(history.join_mut().2.current().incoming, segment(0.5).end);
        let first = *history.first_for_close();
        history.push(&first);
        assert_eq!(
            history.join_mut().2.current().incoming,
            line_differentials(point(9.0, 0.0), first.position).end
        );
        let second = *history.second_for_close().unwrap();
        history.push(&second);
        assert_eq!(
            history.join_mut().2.current(),
            EndpointDifferentials::default()
        );
    }

    #[test]
    fn explicit_closure_keeps_the_incoming_curve() {
        let mut storage = DifferentialHistory::default();
        let mut history = EndpointHistory::new(&mut storage, true);
        history.push(&endpoint(0.0));
        history.record_segment(|| segment(0.25));
        history.push(&endpoint(1.0));
        history.capture_firsts();
        history.record_segment(|| segment(0.75));
        history.push(&endpoint(0.0));
        history.first_for_close();
        let current = history.join_mut().2.current();
        assert_eq!(current.incoming, segment(0.75).end);
        assert_eq!(current.outgoing, segment(0.25).start);
    }

    fn endpoint(x: f32) -> EndpointData {
        EndpointData {
            position: point(x, 0.0),
            src: VertexSource::Endpoint { id: EndpointId(0) },
            ..EndpointData::default()
        }
    }

    fn segment(curvature: f64) -> SegmentDifferentials {
        SegmentDifferentials {
            start: EndpointDifferential::Regular {
                unit_tangent: vector(1.0, 0.0),
                curvature,
            },
            end: EndpointDifferential::Regular {
                unit_tangent: vector(0.0, 1.0),
                curvature: -curvature,
            },
        }
    }
}
