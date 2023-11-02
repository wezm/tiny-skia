// Copyright 2006 The Android Open Source Project
// Copyright 2020 Yevhenii Reizner
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// NOTE: this is not SkPathBuilder, but rather a reimplementation of SkPath.

use alloc::vec;
use alloc::vec::Vec;

use crate::{Path, Point, Rect, Transform, SCALAR_SIN_COS_NEARLY_ZERO};

use crate::path::PathVerb;
use crate::path_geometry;
use crate::path_geometry::Conic;
use crate::scalar::{Scalar, SCALAR_ROOT_2_OVER_2};

const MAX_CONICS_FOR_ARC: usize = 5;

#[derive(Copy, Clone, PartialEq, Debug)]
pub(crate) enum PathDirection {
    /// Clockwise direction for adding closed contours.
    CW,
    /// Counter-clockwise direction for adding closed contours.
    CCW,
}

/// A path builder.
#[derive(Clone, Default, Debug)]
pub struct PathBuilder {
    pub(crate) verbs: Vec<PathVerb>,
    pub(crate) points: Vec<Point>,
    pub(crate) last_move_to_index: usize,
    pub(crate) move_to_required: bool,
}

impl PathBuilder {
    /// Creates a new builder.
    pub fn new() -> Self {
        PathBuilder {
            verbs: Vec::new(),
            points: Vec::new(),
            last_move_to_index: 0,
            move_to_required: true,
        }
    }

    /// Creates a new builder with a specified capacity.
    ///
    /// Number of points depends on a verb type:
    ///
    /// - Move - 1
    /// - Line - 1
    /// - Quad - 2
    /// - Cubic - 3
    /// - Close - 0
    pub fn with_capacity(verbs_capacity: usize, points_capacity: usize) -> Self {
        PathBuilder {
            verbs: Vec::with_capacity(verbs_capacity),
            points: Vec::with_capacity(points_capacity),
            last_move_to_index: 0,
            move_to_required: true,
        }
    }

    /// Creates a new `Path` from `Rect`.
    ///
    /// Never fails since `Rect` is always valid.
    ///
    /// Segments are created clockwise: TopLeft -> TopRight -> BottomRight -> BottomLeft
    ///
    /// The contour is closed.
    pub fn from_rect(rect: Rect) -> Path {
        let verbs = vec![
            PathVerb::Move,
            PathVerb::Line,
            PathVerb::Line,
            PathVerb::Line,
            PathVerb::Close,
        ];

        let points = vec![
            Point::from_xy(rect.left(), rect.top()),
            Point::from_xy(rect.right(), rect.top()),
            Point::from_xy(rect.right(), rect.bottom()),
            Point::from_xy(rect.left(), rect.bottom()),
        ];

        Path {
            bounds: rect,
            verbs,
            points,
        }
    }

    /// Creates a new `Path` from a circle.
    ///
    /// See [`PathBuilder::push_circle`] for details.
    pub fn from_circle(cx: f32, cy: f32, radius: f32) -> Option<Path> {
        let mut b = PathBuilder::new();
        b.push_circle(cx, cy, radius);
        b.finish()
    }

    /// Creates a new `Path` from an oval.
    ///
    /// See [`PathBuilder::push_oval`] for details.
    pub fn from_oval(oval: Rect) -> Option<Path> {
        let mut b = PathBuilder::new();
        b.push_oval(oval);
        b.finish()
    }

    pub(crate) fn reserve(&mut self, additional_verbs: usize, additional_points: usize) {
        self.verbs.reserve(additional_verbs);
        self.points.reserve(additional_points);
    }

    /// Returns the current number of segments in the builder.
    pub fn len(&self) -> usize {
        self.verbs.len()
    }

    /// Checks if the builder has any segments added.
    pub fn is_empty(&self) -> bool {
        self.verbs.is_empty()
    }

    /// Adds beginning of a contour.
    ///
    /// Multiple continuous MoveTo segments are not allowed.
    /// If the previous segment was also MoveTo, it will be overwritten with the current one.
    pub fn move_to(&mut self, x: f32, y: f32) {
        if let Some(PathVerb::Move) = self.verbs.last() {
            let last_idx = self.points.len() - 1;
            self.points[last_idx] = Point::from_xy(x, y);
        } else {
            self.last_move_to_index = self.points.len();
            self.move_to_required = false;

            self.verbs.push(PathVerb::Move);
            self.points.push(Point::from_xy(x, y));
        }
    }

    fn inject_move_to_if_needed(&mut self) {
        if self.move_to_required {
            match self.points.get(self.last_move_to_index).cloned() {
                Some(p) => self.move_to(p.x, p.y),
                None => self.move_to(0.0, 0.0),
            }
        }
    }

    /// Adds a line from the last point.
    ///
    /// - If `Path` is empty - adds Move(0, 0) first.
    /// - If `Path` ends with Close - adds Move(last_x, last_y) first.
    pub fn line_to(&mut self, x: f32, y: f32) {
        self.inject_move_to_if_needed();

        self.verbs.push(PathVerb::Line);
        self.points.push(Point::from_xy(x, y));
    }

    /// Adds a quad curve from the last point to `x`, `y`.
    ///
    /// - If `Path` is empty - adds Move(0, 0) first.
    /// - If `Path` ends with Close - adds Move(last_x, last_y) first.
    pub fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        self.inject_move_to_if_needed();

        self.verbs.push(PathVerb::Quad);
        self.points.push(Point::from_xy(x1, y1));
        self.points.push(Point::from_xy(x, y));
    }

    pub(crate) fn quad_to_pt(&mut self, p1: Point, p: Point) {
        self.quad_to(p1.x, p1.y, p.x, p.y);
    }

    // We do not support conic segments, but Skia still relies on them from time to time.
    // This method will simply convert the input data into quad segments.
    pub(crate) fn conic_to(&mut self, x1: f32, y1: f32, x: f32, y: f32, weight: f32) {
        // check for <= 0 or NaN with this test
        if !(weight > 0.0) {
            self.line_to(x, y);
        } else if !weight.is_finite() {
            self.line_to(x1, y1);
            self.line_to(x, y);
        } else if weight == 1.0 {
            self.quad_to(x1, y1, x, y);
        } else {
            self.inject_move_to_if_needed();

            let last = self.last_point().unwrap();
            let quadder = path_geometry::AutoConicToQuads::compute(
                last,
                Point::from_xy(x1, y1),
                Point::from_xy(x, y),
                weight,
            );
            if let Some(quadder) = quadder {
                // Points are ordered as: 0 - 1 2 - 3 4 - 5 6 - ..
                // `count` is a number of pairs +1
                let mut offset = 1;
                for _ in 0..quadder.len {
                    let pt1 = quadder.points[offset + 0];
                    let pt2 = quadder.points[offset + 1];
                    self.quad_to(pt1.x, pt1.y, pt2.x, pt2.y);
                    offset += 2;
                }
            }
        }
    }

    pub(crate) fn conic_points_to(&mut self, pt1: Point, pt2: Point, weight: f32) {
        self.conic_to(pt1.x, pt1.y, pt2.x, pt2.y, weight);
    }

    /// Adds a cubic curve from the last point to `x`, `y`.
    ///
    /// - If `Path` is empty - adds Move(0, 0) first.
    /// - If `Path` ends with Close - adds Move(last_x, last_y) first.
    pub fn cubic_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        self.inject_move_to_if_needed();

        self.verbs.push(PathVerb::Cubic);
        self.points.push(Point::from_xy(x1, y1));
        self.points.push(Point::from_xy(x2, y2));
        self.points.push(Point::from_xy(x, y));
    }

    pub(crate) fn cubic_to_pt(&mut self, p1: Point, p2: Point, p: Point) {
        self.cubic_to(p1.x, p1.y, p2.x, p2.y, p.x, p.y);
    }

    /// Adds a arc contour bounded by the provided rectangle and angles.
    ///
    /// Arc added is part of ellipse bounded by oval, from startAngle through sweepAngle. Both startAngle and sweepAngle are measured in degrees, where zero degrees is aligned with the positive x-axis, and positive sweeps extends arc clockwise.
    pub fn arc_to(
        &mut self,
        oval: Rect,
        start_angle_deg: f32,
        sweep_angle_deg: f32,
        mut force_move_to: bool,
    ) {
        if oval.width() < 0. || oval.height() < 0. {
            return;
        }

        if self.verbs.is_empty() {
            force_move_to = true;
        }

        if let Some(lone_point) = arc_is_lone_point(oval, start_angle_deg, sweep_angle_deg) {
            if force_move_to {
                self.move_to(lone_point.x, lone_point.y);
            } else {
                self.line_to(lone_point.x, lone_point.y);
            }
            return;
        }

        let (start_v, stop_v, dir) = angles_to_unit_vectors(start_angle_deg, sweep_angle_deg);

        // Adds a move-to to 'pt' if forceMoveTo is true. Otherwise a lineTo unless we're sufficiently
        // close to 'pt' currently. This prevents spurious lineTos when adding a series of contiguous
        // arcs from the same oval.
        // FIXME: This closure is of dubious value
        let add_pt = |this: &mut PathBuilder, point: Point| {
            if force_move_to {
                this.move_to(point.x, point.y)
            } else if !nearly_equal(this.last_point(), point) {
                this.line_to(point.x, point.y)
            }
        };

        // At this point, we know that the arc is not a lone point, but startV == stopV
        // indicates that the sweepAngle is too small such that angles_to_unit_vectors
        // cannot handle it.
        if start_v == stop_v {
            let end_angle = (start_angle_deg + sweep_angle_deg).to_radians();
            let radius_x = oval.width().half();
            let radius_y = oval.height().half();
            // We do not use SkScalar[Sin|Cos]SnapToZero here. When sin(startAngle) is 0 and sweepAngle
            // is very small and radius is huge, the expected behavior here is to draw a line. But
            // calling SkScalarSinSnapToZero will make sin(endAngle) be 0 which will then draw a dot.
            // let center_x = oval.left().half() + oval.right().half();
            // let center_y = oval.top().half() + oval.bottom().half();
            let single_pt = Point::from_xy(
                oval.center_x() + radius_x * end_angle.cos(),
                oval.center_y() + radius_y + end_angle.cos(),
            );
            add_pt(self, single_pt);
            return;
        }

        let mut single_pt = Point::default(); // FIXME: return this from build_arc_conics
        let mut conics = [Conic::default(); MAX_CONICS_FOR_ARC];
        let count = build_arc_conics(oval, start_v, stop_v, dir, &mut conics, &mut single_pt);
        if count > 0 {
            self.reserve(0, count * 2 + 1);
            let pt = &conics[0].points[0];
            add_pt(self, *pt);
            for i in 0..count {
                self.conic_points_to(conics[i].points[1], conics[i].points[2], conics[i].weight);
            }
        } else {
            add_pt(self, single_pt);
        }
    }

    /// Closes the current contour.
    ///
    /// A closed contour connects the first and the last Point
    /// with a line, forming a continuous loop.
    ///
    /// Does nothing when `Path` is empty or already closed.
    ///
    /// Open and closed contour will be filled the same way.
    /// Stroking an open contour will add LineCap at contour's start and end.
    /// Stroking an closed contour will add LineJoin at contour's start and end.
    pub fn close(&mut self) {
        // don't add a close if it's the first verb or a repeat
        if !self.verbs.is_empty() {
            if self.verbs.last().cloned() != Some(PathVerb::Close) {
                self.verbs.push(PathVerb::Close);
            }
        }

        self.move_to_required = true;
    }

    /// Returns the last point if any.
    pub fn last_point(&self) -> Option<Point> {
        self.points.last().cloned()
    }

    pub(crate) fn set_last_point(&mut self, pt: Point) {
        match self.points.last_mut() {
            Some(last) => *last = pt,
            None => self.move_to(pt.x, pt.y),
        }
    }

    pub(crate) fn is_zero_length_since_point(&self, start_pt_index: usize) -> bool {
        let count = self.points.len() - start_pt_index;
        if count < 2 {
            return true;
        }

        let first = self.points[start_pt_index];
        for i in 1..count {
            if first != self.points[start_pt_index + i] {
                return false;
            }
        }

        true
    }

    /// Adds a rectangle contour.
    ///
    /// The contour is closed and has a clock-wise direction.
    pub fn push_rect(&mut self, rect: Rect) {
        self.move_to(rect.left(), rect.top());
        self.line_to(rect.right(), rect.top());
        self.line_to(rect.right(), rect.bottom());
        self.line_to(rect.left(), rect.bottom());
        self.close();
    }

    /// Adds an oval contour bounded by the provided rectangle.
    ///
    /// The contour is closed and has a clock-wise direction.
    pub fn push_oval(&mut self, oval: Rect) {
        let cx = oval.left().half() + oval.right().half();
        let cy = oval.top().half() + oval.bottom().half();

        let oval_points = [
            Point::from_xy(cx, oval.bottom()),
            Point::from_xy(oval.left(), cy),
            Point::from_xy(cx, oval.top()),
            Point::from_xy(oval.right(), cy),
        ];

        let rect_points = [
            Point::from_xy(oval.right(), oval.bottom()),
            Point::from_xy(oval.left(), oval.bottom()),
            Point::from_xy(oval.left(), oval.top()),
            Point::from_xy(oval.right(), oval.top()),
        ];

        let weight = SCALAR_ROOT_2_OVER_2;
        self.move_to(oval_points[3].x, oval_points[3].y);
        for (p1, p2) in rect_points.iter().zip(oval_points.iter()) {
            self.conic_points_to(*p1, *p2, weight);
        }
        self.close();
    }

    /// Adds a circle contour.
    ///
    /// The contour is closed and has a clock-wise direction.
    ///
    /// Does nothing when:
    /// - `radius` <= 0
    /// - any value is not finite or really large
    pub fn push_circle(&mut self, x: f32, y: f32, r: f32) {
        if let Some(r) = Rect::from_xywh(x - r, y - r, r + r, r + r) {
            self.push_oval(r);
        }
    }

    /// Adds a path.
    pub fn push_path(&mut self, other: &Path) {
        self.last_move_to_index = self.points.len();

        self.verbs.extend_from_slice(&other.verbs);
        self.points.extend_from_slice(&other.points);
    }

    pub(crate) fn push_path_builder(&mut self, other: &PathBuilder) {
        if other.is_empty() {
            return;
        }

        if self.last_move_to_index != 0 {
            self.last_move_to_index = self.points.len() + other.last_move_to_index;
        }

        self.verbs.extend_from_slice(&other.verbs);
        self.points.extend_from_slice(&other.points);
    }

    /// Appends, in a reverse order, the first contour of path ignoring path's last point.
    pub(crate) fn reverse_path_to(&mut self, other: &PathBuilder) {
        if other.is_empty() {
            return;
        }

        debug_assert_eq!(other.verbs[0], PathVerb::Move);

        let mut points_offset = other.points.len() - 1;
        for verb in other.verbs.iter().rev() {
            match verb {
                PathVerb::Move => {
                    // if the path has multiple contours, stop after reversing the last
                    break;
                }
                PathVerb::Line => {
                    // We're moving one point back manually, to prevent points_offset overflow.
                    let pt = other.points[points_offset - 1];
                    points_offset -= 1;
                    self.line_to(pt.x, pt.y);
                }
                PathVerb::Quad => {
                    let pt1 = other.points[points_offset - 1];
                    let pt2 = other.points[points_offset - 2];
                    points_offset -= 2;
                    self.quad_to(pt1.x, pt1.y, pt2.x, pt2.y);
                }
                PathVerb::Cubic => {
                    let pt1 = other.points[points_offset - 1];
                    let pt2 = other.points[points_offset - 2];
                    let pt3 = other.points[points_offset - 3];
                    points_offset -= 3;
                    self.cubic_to(pt1.x, pt1.y, pt2.x, pt2.y, pt3.x, pt3.y);
                }
                PathVerb::Close => {}
            }
        }
    }

    /// Reset the builder.
    ///
    /// Memory is not deallocated.
    pub fn clear(&mut self) {
        self.verbs.clear();
        self.points.clear();
        self.last_move_to_index = 0;
        self.move_to_required = true;
    }

    /// Finishes the builder and returns a `Path`.
    ///
    /// Returns `None` when `Path` is empty or has invalid bounds.
    pub fn finish(self) -> Option<Path> {
        if self.is_empty() {
            return None;
        }

        // Just a move to? Bail.
        if self.verbs.len() == 1 {
            return None;
        }

        let bounds = Rect::from_points(&self.points)?;

        Some(Path {
            bounds,
            verbs: self.verbs,
            points: self.points,
        })
    }
}

fn arc_is_lone_point(oval: Rect, start_angle: f32, sweep_angle: f32) -> Option<Point> {
    if sweep_angle == 0. && (start_angle == 0. || start_angle == 360.) {
        return Some(Point::from_xy(oval.right(), oval.center_y()));
    } else if oval.width() == 0. && oval.height() == 0. {
        return Some(Point::from_xy(oval.right(), oval.top()));
    }
    None
}

/// Return the unit vectors pointing at the start/stop points for the given start/sweep angles
fn angles_to_unit_vectors(start_angle: f32, sweep_angle: f32) -> (Point, Point, PathDirection) {
    let start_rad = start_angle.to_radians();
    let mut stop_rad = (start_angle + sweep_angle).to_radians();

    let start_v = Point::from_xy(cos_snap_to_zero(start_rad), sin_snap_to_zero(start_rad));
    let mut stop_v = Point::from_xy(cos_snap_to_zero(stop_rad), sin_snap_to_zero(stop_rad));

    /*  If the sweep angle is nearly (but less than) 360, then due to precision
    loss in radians-conversion and/or sin/cos, we may end up with coincident
    vectors, which will fool SkBuildQuadArc into doing nothing (bad) instead
    of drawing a nearly complete circle (good).
    e.g. canvas.drawArc(0, 359.99, ...)
    -vs- canvas.drawArc(0, 359.9, ...)
    We try to detect this edge case, and tweak the stop vector
    */
    if start_v == stop_v {
        let sw = sweep_angle.abs();
        if sw < 360. && sw > 359. {
            // make a guess at a tiny angle (in radians) to tweak by
            let delta_rad = (1f32 / 512.).copysign(sweep_angle);
            // not sure how much will be enough, so we use a loop
            loop {
                stop_rad -= delta_rad;
                stop_v.x = cos_snap_to_zero(stop_rad);
                stop_v.y = sin_snap_to_zero(stop_rad);
                if start_v != stop_v {
                    break;
                }
            }
        }
    }
    let dir = if sweep_angle > 0. {
        PathDirection::CW
    } else {
        PathDirection::CCW
    };
    (start_v, stop_v, dir)
}

#[inline]
fn cos_snap_to_zero(radians: f32) -> f32 {
    let v = radians.cos();
    if v.is_nearly_zero_within_tolerance(SCALAR_SIN_COS_NEARLY_ZERO) {
        0.
    } else {
        v
    }
}

#[inline]
fn sin_snap_to_zero(radians: f32) -> f32 {
    let v = radians.sin();
    if v.is_nearly_zero_within_tolerance(SCALAR_SIN_COS_NEARLY_ZERO) {
        0.
    } else {
        v
    }
}

fn build_arc_conics(
    oval: Rect,
    start: Point,
    stop: Point,
    dir: PathDirection,
    conics: &mut [Conic; MAX_CONICS_FOR_ARC],
    single_pt: &mut Point,
) -> usize {
    let mut matrix = Transform::from_scale(oval.width().half(), oval.height().half())
        .post_translate(oval.center_x(), oval.center_y());

    match Conic::build_unit_arc(start, stop, dir, matrix, conics) {
        Some(conics) => conics.len(),
        None => {
            let mut result = stop;
            matrix.map_point(&mut result);
            *single_pt = result;
            0
        }
    }
}

fn nearly_equal(a: Option<Point>, b: Point) -> bool {
    a.map_or(false, |a| {
        a.x.is_nearly_equal(b.x) && a.y.is_nearly_equal(b.y)
    })
}
