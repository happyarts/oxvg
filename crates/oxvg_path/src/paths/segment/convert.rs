use std::{f64::consts::PI, iter::Peekable};

use crate::{
    command::{self, ID},
    geometry::{
        Arc, Curve, Point, QuadraticBezierTo, SmoothBezierTo, Tolerance, TolerancePrecision,
        ToleranceSquared,
    },
    math::{self},
    paths::segment::{Data, IterStartCursorItem, Path, Segment},
};

/// The control point a following smooth command may reflect, with the kind of
/// curve it belongs to: `S` reflects only a cubic's last control point, `T`
/// only a quadratic's. After any other command, theirs is the current point.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Control {
    Cubic(Point),
    Quadratic(Point),
}

impl Control {
    /// The control point `S` reflects after `control`.
    fn cubic(control: Option<Self>) -> Option<Point> {
        match control {
            Some(Self::Cubic(point)) => Some(point),
            _ => None,
        }
    }

    /// The control point `T` reflects after `control`.
    fn quadratic(control: Option<Self>) -> Option<Point> {
        match control {
            Some(Self::Quadratic(point)) => Some(point),
            _ => None,
        }
    }
}

impl Data {
    #[allow(clippy::too_many_lines)]
    pub(crate) fn take(
        command: &command::Data,
        start: &mut Point,
        cursor: &mut Point,
        control: &mut Option<Control>,
        z_end: &mut bool,
    ) -> Option<Self> {
        let last_control = control.take();
        match command {
            command::Data::MoveBy(a) => {
                *cursor += Point::from(*a);
                *start = *cursor;
                None
            }
            command::Data::MoveTo(a) => {
                *cursor = Point::from(*a);
                *start = *cursor;
                None
            }
            command::Data::LineBy(a) => {
                *cursor += Point::from(*a);
                Some(Data::LineTo(*cursor))
            }
            command::Data::LineTo(a) => {
                *cursor = Point::from(*a);
                Some(Data::LineTo(*cursor))
            }
            command::Data::HorizontalLineBy(a) => {
                cursor.x += a[0];
                Some(Data::LineTo(*cursor))
            }
            command::Data::HorizontalLineTo(a) => {
                cursor.x = a[0];
                Some(Data::LineTo(*cursor))
            }
            command::Data::VerticalLineBy(a) => {
                cursor.y += a[0];
                Some(Data::LineTo(*cursor))
            }
            command::Data::VerticalLineTo(a) => {
                cursor.y = a[0];
                Some(Data::LineTo(*cursor))
            }
            command::Data::CubicBezierBy(a) => {
                let start_control = *cursor + Point::new(a[0], a[1]);
                let end_control = *cursor + Point::new(a[2], a[3]);
                let end_point = *cursor + Point::new(a[4], a[5]);

                *control = Some(Control::Cubic(end_control));
                *cursor = end_point;
                Some(Self::CurveTo(Curve::new(
                    start_control,
                    end_control,
                    end_point,
                )))
            }
            command::Data::CubicBezierTo(a) => {
                let end_control = Point::new(a[2], a[3]);
                let end_point = Point::new(a[4], a[5]);

                *control = Some(Control::Cubic(end_control));
                *cursor = end_point;
                Some(Data::CurveTo(Curve::from(*a)))
            }
            command::Data::QuadraticBezierBy(a) => {
                let quad_control = *cursor + Point::new(a[0], a[1]);
                let end = *cursor + Point::new(a[2], a[3]);
                *control = Some(Control::Quadratic(quad_control));

                let start = *cursor;
                *cursor = end;
                Some(Data::CurveTo(Curve::new_quad(quad_control, start, end)))
            }
            command::Data::QuadraticBezierTo(a) => {
                let quad_control = Point::new(a[0], a[1]);
                let end = Point::new(a[2], a[3]);
                *control = Some(Control::Quadratic(quad_control));

                let start = *cursor;
                *cursor = end;
                Some(Data::CurveTo(Curve::new_quad(quad_control, start, end)))
            }
            command::Data::SmoothBezierBy(a) => {
                let start_control =
                    Control::cubic(last_control).map_or(*cursor, |c| c.reflect(*cursor));
                let end_control = *cursor + Point::new(a[0], a[1]);
                let end = *cursor + Point::new(a[2], a[3]);

                *control = Some(Control::Cubic(end_control));
                *cursor = end;
                Some(Data::CurveTo(Curve::new(start_control, end_control, end)))
            }
            command::Data::SmoothBezierTo(a) => {
                let start_control =
                    Control::cubic(last_control).map_or(*cursor, |c| c.reflect(*cursor));
                let end_control = Point::new(a[0], a[1]);
                let end = Point::new(a[2], a[3]);

                *control = Some(Control::Cubic(end_control));
                *cursor = end;
                Some(Data::CurveTo(Curve::new(start_control, end_control, end)))
            }
            command::Data::SmoothQuadraticBezierBy(a) => {
                let start_control =
                    Control::quadratic(last_control).map_or(*cursor, |c| c.reflect(*cursor));
                let end = *cursor + Point::from(*a);

                *control = Some(Control::Quadratic(start_control));
                let start = *cursor;
                *cursor = end;
                Some(Data::CurveTo(Curve::new_quad(start_control, start, end)))
            }
            command::Data::SmoothQuadraticBezierTo(a) => {
                let start_control =
                    Control::quadratic(last_control).map_or(*cursor, |c| c.reflect(*cursor));
                let end = Point::from(*a);

                *control = Some(Control::Quadratic(start_control));
                let start = *cursor;
                *cursor = end;
                Some(Data::CurveTo(Curve::new_quad(start_control, start, end)))
            }
            command::Data::ArcBy(a) => Some(
                Arc::from_arc_by(*a, cursor)
                    .map(Self::ArcTo)
                    .unwrap_or_else(|| {
                        *cursor += Point::new(a[5], a[6]);
                        Self::LineTo(*cursor)
                    }),
            ),
            command::Data::ArcTo(a) => Some(
                Arc::from_arc_to(*a, cursor)
                    .map(Self::ArcTo)
                    .unwrap_or_else(|| {
                        *cursor = Point::new(a[5], a[6]);
                        Self::LineTo(*cursor)
                    }),
            ),
            command::Data::ClosePath => {
                *z_end = true;
                None
            }
            command::Data::Implicit(command) => {
                *control = last_control;
                Self::take(command, start, cursor, control, z_end)
            }
        }
    }
}

impl Segment {
    fn take<'a>(
        iterator: &mut Peekable<impl Iterator<Item = &'a command::Data>>,
        start: &mut Point,
        cursor: &mut Point,
        tolerance: &Tolerance,
    ) -> Self {
        let mut result = Segment {
            start: *start,
            data: vec![],
            closed: false,
        };
        let mut z_end = false;
        let mut control = None;
        for command in iterator.by_ref() {
            if let Some(data) = Data::take(command, start, cursor, &mut control, &mut z_end) {
                result.data.push(data);
            } else {
                break;
            }
        }
        if z_end {
            // A `Z` right after another one closes nothing
            while iterator
                .next_if(|command| matches!(command.as_explicit(), command::Data::ClosePath))
                .is_some()
            {}
            *cursor = result.start;
            result.closed = true;
            // A move right after `Z` starts the next segment: it sets `start` and `cursor`
            if let Some(command) = iterator.next_if(|command| {
                matches!(command, command::Data::MoveTo(_) | command::Data::MoveBy(_))
            }) {
                Data::take(command, start, cursor, &mut control, &mut z_end);
            }
            return result;
        }
        if result.data.is_empty() {
            return result;
        }

        let tolerance_squared = tolerance.positional * tolerance.positional;
        // NOTE: `start` may already be the start of the next segment
        result.closed = result
            .start
            .distance_squared(result.data().last().unwrap().end_point())
            < tolerance_squared;
        result
    }
}

impl Path {
    /// Returns the segment path from a parsed svg path.
    pub fn from_svg(value: &crate::Path, tolerance: &Tolerance) -> Self {
        let mut result = Path(vec![]);
        let mut iterator = value.0.iter().peekable();
        let mut start = match iterator.next() {
            Some(command::Data::MoveTo(p) | command::Data::MoveBy(p)) => Point::from(*p),
            None => return result,
            _ => unreachable!("Path data starts with non-move command"),
        };
        let mut cursor = start;
        while iterator.peek().is_some() {
            let component = Segment::take(&mut iterator, &mut start, &mut cursor, tolerance);
            result.0.push(component);
        }
        // A move at the end starts a segment without data, as does a path of only a move
        if result.0.is_empty()
            || matches!(
                value.0.last(),
                Some(command::Data::MoveTo(_) | command::Data::MoveBy(_))
            )
        {
            result.0.push(Segment {
                start,
                data: vec![],
                closed: false,
            });
        }
        result
    }
}

impl Path {
    /// Returns an svg equivalent path from a segmented path, in the compactest set of equivalent commands.
    pub fn to_svg(&self, tolerance: &Tolerance, smart_arc_rounding: bool) -> crate::Path {
        // NOTE: When comparing compactness, order of precedence is given by order of TO/BY commands.
        //       We should order it so that BY is preferenced for readability.
        let tolerance_squared = tolerance.square();
        let precision = tolerance.precision();

        let mut commands: Vec<command::Data> = vec![];

        let mut last_control = None;
        let mut reader = Reader::default();
        for IterStartCursorItem {
            segment_start,
            cursor: start,
            data,
            next,
            command,
            close,
            ..
        } in self.iter_start_cursor()
        {
            // A subpath's first curve has no previous control point.
            let control = last_control.take().filter(|_| command > 0);
            let Some(data) = data else {
                let m = Self::to_svg_move(segment_start, reader.point, precision);
                reader.push(&mut commands, m);
                if close {
                    reader.push(&mut commands, command::Data::ClosePath);
                }
                continue;
            };
            let is_start = command == 0;
            let previous = if is_start { None } else { commands.last() };
            let implicit = previous.map(|d| d.id().next_implicit());
            // Relative commands go from where the reader is: after the move, at the start
            let origin = if is_start {
                Self::rounded(segment_start, precision)
            } else {
                reader.point
            };

            let mut command = match data {
                Data::LineTo(to) => Self::to_svg_line(
                    previous,
                    segment_start,
                    CommandStart {
                        exact: start,
                        read: origin,
                    },
                    *to,
                    implicit.as_ref(),
                    precision,
                    close,
                ),
                Data::ArcTo(arc) => Self::to_svg_arc(
                    previous,
                    arc,
                    CommandStart {
                        exact: start,
                        read: origin,
                    },
                    implicit.as_ref(),
                    tolerance,
                    tolerance_squared,
                    precision,
                    smart_arc_rounding,
                ),
                Data::CurveTo(curve) => {
                    let (command, control) = Self::to_svg_curve(
                        previous,
                        control,
                        CommandStart {
                            exact: start,
                            read: origin,
                        },
                        curve,
                        next,
                        implicit.as_ref(),
                        tolerance_squared,
                        precision,
                    );
                    last_control = control;
                    command
                }
            };

            if is_start {
                let (move_to, move_by) = Self::to_svg_moves(segment_start, reader.point, precision);
                let m = match command.id().as_explicit() {
                    ID::LineBy => {
                        command = command::Data::Implicit(Box::new(command));
                        move_by
                    }
                    ID::LineTo => {
                        command = command::Data::Implicit(Box::new(command));
                        move_to
                    }
                    _ => compactest(None, move_to, move_by, None, precision),
                };
                reader.push(&mut commands, m);
            }
            let close = close && command.id() != ID::ClosePath;
            reader.push(&mut commands, command);
            if close {
                reader.push(&mut commands, command::Data::ClosePath);
            }
        }

        crate::Path(commands)
    }

    /// `point`, rounded.
    fn rounded(point: Point, precision: TolerancePrecision) -> Point {
        Point::new(precision.round(point.x), precision.round(point.y))
    }

    /// `M` and `m` to the rounded start of a segment, `m` from where the reader is, so
    /// both go to the same point.
    fn to_svg_moves(
        segment_start: Point,
        reader: Point,
        precision: TolerancePrecision,
    ) -> (command::Data, command::Data) {
        let to = Self::rounded(segment_start, precision);
        let mut by = command::Data::MoveBy((to - reader).into());
        by.round(precision);
        (command::Data::MoveTo(to.into()), by)
    }

    /// The shorter of `M` and `m` to the rounded start of a segment; `M` when both are as long.
    fn to_svg_move(
        segment_start: Point,
        reader: Point,
        precision: TolerancePrecision,
    ) -> command::Data {
        let (to, by) = Self::to_svg_moves(segment_start, reader, precision);
        compactest(None, to, by, None, precision)
    }

    fn to_svg_line(
        previous: Option<&command::Data>,
        segment_start: Point,
        from: CommandStart,
        to: Point,
        implicit: Option<&ID>,
        precision: TolerancePrecision,
        close: bool,
    ) -> command::Data {
        let by = from.relative(to, precision);
        // A line of length zero is the `Z` that closes its segment only as the
        // segment's last command; elsewhere `Z` would move the current point.
        if from.exact == to && close {
            command::Data::ClosePath
        } else if precision.round(to.x) == precision.round(from.read.x) {
            let v_to = command::Data::VerticalLineTo([to.y]);
            let v_by = command::Data::VerticalLineBy([by.y]);
            compactest(previous, v_by, v_to, implicit, precision)
        } else if precision.round(to.y) == precision.round(from.read.y) {
            let h_to = command::Data::HorizontalLineTo([to.x]);
            let h_by = command::Data::HorizontalLineBy([by.x]);
            compactest(previous, h_by, h_to, implicit, precision)
        } else if to == segment_start {
            command::Data::ClosePath
        } else {
            let l_to = command::Data::LineTo(to.into());
            let l_by = command::Data::LineBy(by.into());
            compactest(previous, l_by, l_to, implicit, precision)
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::too_many_lines)]
    pub(crate) fn to_svg_curve(
        previous: Option<&command::Data>,
        control: Option<Control>,
        from: CommandStart,
        curve: &Curve,
        next: Option<&Data>,
        implicit: Option<&ID>,
        tolerance_squared: ToleranceSquared,
        precision: TolerancePrecision,
    ) -> (command::Data, Option<Control>) {
        let start_control = curve.start_control;
        let end_control = curve.end_control;
        let to = curve.end_point;
        let [by, start_control_by, end_control_by] =
            [to, start_control, end_control].map(|point| from.relative(point, precision));

        let mut candidates = Vec::with_capacity(4);

        let smooth_bezier =
            curve.smooth_bezier(from.exact, Control::cubic(control), tolerance_squared);
        let quadratic_control = Control::quadratic(control);
        let (_, _, quadratic_bezier, smooth_quadratic_bezier) =
            curve.types(from.exact, quadratic_control, tolerance_squared);
        if let Some(SmoothBezierTo {
            end_control,
            end_point: _,
        }) = smooth_bezier
        {
            candidates.push(command::Data::SmoothBezierBy([
                end_control_by.x,
                end_control_by.y,
                by.x,
                by.y,
            ]));
            candidates.push(command::Data::SmoothBezierTo([
                end_control.x,
                end_control.y,
                to.x,
                to.y,
            ]));
        } else {
            candidates.push(command::Data::CubicBezierBy([
                start_control_by.x,
                start_control_by.y,
                end_control_by.x,
                end_control_by.y,
                by.x,
                by.y,
            ]));
            candidates.push(command::Data::CubicBezierTo([
                start_control.x,
                start_control.y,
                end_control.x,
                end_control.y,
                to.x,
                to.y,
            ]));
        }

        if smooth_quadratic_bezier.is_some() || quadratic_bezier.is_some() {
            let next_is_smooth_quadratic_bezier =
                quadratic_bezier.as_ref().is_some_and(|q| match next {
                    Some(Data::CurveTo(next)) => {
                        next.quad_control(to, tolerance_squared).is_some_and(|c| {
                            next.smooth_quadratic_bezier_unchecked_quad(
                                to,
                                Some(q.quad_control),
                                c,
                                tolerance_squared,
                            )
                            .is_some()
                        })
                    }
                    _ => false,
                });
            if next_is_smooth_quadratic_bezier {
                candidates.clear();
            }
        }
        if smooth_quadratic_bezier.is_some() {
            candidates.push(command::Data::SmoothQuadraticBezierBy(by.into()));
            candidates.push(command::Data::SmoothQuadraticBezierTo(to.into()));
        } else if let Some(QuadraticBezierTo {
            quad_control,
            end_point: _,
        }) = quadratic_bezier
        {
            let quad_control_by = from.relative(quad_control, precision);
            candidates.push(command::Data::QuadraticBezierBy([
                quad_control_by.x,
                quad_control_by.y,
                by.x,
                by.y,
            ]));
            candidates.push(command::Data::QuadraticBezierTo([
                quad_control.x,
                quad_control.y,
                to.x,
                to.y,
            ]));
        }

        let result = compactest_vec(previous, candidates, implicit, precision);
        let control = match result.id().as_explicit() {
            ID::QuadraticBezierTo | ID::QuadraticBezierBy => {
                quadratic_bezier.map(|q| Control::Quadratic(q.quad_control))
            }
            ID::SmoothQuadraticBezierTo | ID::SmoothQuadraticBezierBy => {
                quadratic_control.map(|c| Control::Quadratic(c.reflect(from.exact)))
            }
            ID::CubicBezierTo | ID::CubicBezierBy | ID::SmoothBezierBy | ID::SmoothBezierTo => {
                Some(Control::Cubic(end_control))
            }
            _ => None,
        };
        (result, control)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn to_svg_arc(
        previous: Option<&command::Data>,
        arc: &Arc,
        from: CommandStart,
        implicit: Option<&ID>,
        tolerance: &Tolerance,
        tolerance_squared: ToleranceSquared,
        precision: TolerancePrecision,
        smart_arc_rounding: bool,
    ) -> command::Data {
        let by = from.relative(arc.end_point(), precision);
        let mut arc_to = arc.to_arc_to(tolerance, tolerance_squared, precision);
        let mut arc_by = arc_to;
        arc_by[5] = by.x;
        arc_by[6] = by.y;

        // An arc whose end rounds onto its start has no saggita to keep
        let ends_at_start = precision.round(by.x) == 0.0 && precision.round(by.y) == 0.0;
        if smart_arc_rounding
            && !ends_at_start
            && let Some(saggita) = math::saggita(&arc_by, tolerance.positional)
        {
            let mut p = precision.0;
            let mut new_arc = arc_by;
            while p >= 1.0 {
                new_arc[0] = TolerancePrecision(p).round(arc_by[0]);
                new_arc[1] = TolerancePrecision(p).round(arc_by[1]);
                p /= 10.0;
                let Some(saggita_new) = math::saggita(&new_arc, tolerance.positional) else {
                    break;
                };
                if (saggita - saggita_new).abs() < tolerance.positional {
                    arc_by = new_arc;
                } else {
                    break;
                }
            }
            arc_to[0] = arc_by[0];
            arc_to[1] = arc_by[1];
        }

        let tangent = start_tangent_unit(arc);
        let min_by = Point::new(
            precision.round(precision.descale(tangent.x)),
            precision.round(precision.descale(tangent.y)),
        );
        if ends_at_start {
            arc_by[5] = min_by.x;
            arc_by[6] = min_by.y;
        } else {
            arc_by[5] = precision.round(arc_by[5]);
            arc_by[6] = precision.round(arc_by[6]);
        }
        // The reader's point is a sum of rounded values: rounded again, it's on the grid
        let origin = Self::rounded(from.read, precision);
        if precision.round(arc_to[5]) == origin.x && precision.round(arc_to[6]) == origin.y {
            arc_to[5] = origin.x + min_by.x;
            arc_to[6] = origin.y + min_by.y;
        } else {
            arc_to[5] = precision.round(arc_to[5]);
            arc_to[6] = precision.round(arc_to[6]);
        }
        let a_by = command::Data::ArcBy(arc_by);
        let a_to = command::Data::ArcTo(arc_to);
        compactest(previous, a_by, a_to, implicit, precision)
    }
}

/// Where a command starts: the point exactly, and where a reader of the written path is.
#[derive(Clone, Copy)]
pub(crate) struct CommandStart {
    pub(crate) exact: Point,
    pub(crate) read: Point,
}

impl CommandStart {
    /// `to` relative to the reader: the distance from the exact point while that still
    /// reads within half a step of `to`, else from where the reader is.
    pub(crate) fn relative(self, to: Point, precision: TolerancePrecision) -> Point {
        let half_step = 0.5 / precision.0;
        let axis = |to: f64, exact: f64, read: f64| {
            let from_exact = precision.round(to - exact);
            if (read + from_exact - to).abs() <= half_step * (1.0 + 1e-9) {
                from_exact
            } else {
                to - read
            }
        };
        Point::new(
            axis(to.x, self.exact.x, self.read.x),
            axis(to.y, self.exact.y, self.read.y),
        )
    }
}

/// Where a reader of written path data is: the start of its subpath and its point.
#[derive(Default)]
struct Reader {
    start: Point,
    point: Point,
}

impl Reader {
    /// Appends `command` to `commands` and moves as reading it does.
    fn push(&mut self, commands: &mut Vec<command::Data>, command: command::Data) {
        let (mut control, mut z_end) = (None, false);
        Data::take(
            &command,
            &mut self.start,
            &mut self.point,
            &mut control,
            &mut z_end,
        );
        if z_end {
            self.point = self.start;
        }
        commands.push(command);
    }
}

fn compactest_vec(
    previous: Option<&command::Data>,
    data: impl IntoIterator<Item = command::Data>,
    implicit: Option<&ID>,
    precision: TolerancePrecision,
) -> command::Data {
    let previous_args = previous.and_then(command::Data::args);
    data.into_iter()
        .map(|d| {
            if implicit.is_some_and(|i| *i == d.id()) {
                command::Data::Implicit(Box::new(d))
            } else {
                d
            }
        })
        .map(|mut d| {
            d.round(precision);
            d
        })
        .map(|d| {
            let args = d.args();
            let hint = d.size_hint_with_args(args.as_ref());
            if previous_args.as_ref().is_some_and(|p| {
                d.is_implicit()
                    && args.is_some_and(|args| args.is_space_needed(p))
                    && !hint.negative
            }) {
                (hint.len + 1, d)
            } else {
                (hint.len, d)
            }
        })
        .min_by(|a, b| {
            a.0.cmp(&b.0)
                .then_with(|| b.1.is_implicit().cmp(&a.1.is_implicit()))
        })
        .map(|d| d.1)
        .unwrap()
}

pub(crate) fn compactest(
    previous: Option<&command::Data>,
    left: command::Data,
    right: command::Data,
    implicit: Option<&ID>,
    precision: TolerancePrecision,
) -> command::Data {
    compactest_vec(previous, [left, right], implicit, precision)
}

fn start_tangent_unit(arc: &Arc) -> Point {
    let (sin_a, cos_a) = arc.start_angle().sin_cos();
    let mut tx = -arc.radii().x * sin_a;
    let mut ty = arc.radii().y * cos_a;

    let mut effective_sweep = arc.sweep_angle().rem_euclid(2.0 * PI);
    if effective_sweep > PI {
        effective_sweep -= 2.0 * PI;
    }
    if effective_sweep < 0.0 {
        tx = -tx;
        ty = -ty;
    }

    let (sin_r, cos_r) = arc.x_rotation().sin_cos();
    let rx = tx * cos_r - ty * sin_r;
    let ry = tx * sin_r + ty * cos_r;

    let len = (rx * rx + ry * ry).sqrt();
    if len == 0.0 {
        Point::new(0.0, 0.0)
    } else {
        Point::new(rx / len, ry / len)
    }
}

#[cfg(test)]
mod test {
    use oxvg_parse::Parse as _;

    use crate::{
        geometry::{Point, Tolerance},
        optimize::Options,
        paths::segment::{Data, Path, Segment},
    };

    #[test]
    fn lines() {
        let source = "M5,5 L5,15 L15,15 L15,5 L5,5";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);

        assert_eq!(
            path,
            Path(vec![Segment {
                start: Point::splat(5.0),
                data: vec![
                    Data::LineTo(Point::new(5.0, 15.0)),
                    Data::LineTo(Point::new(15.0, 15.0)),
                    Data::LineTo(Point::new(15.0, 5.0)),
                    Data::LineTo(Point::new(5.0, 5.0))
                ],
                closed: true
            }])
        );

        let path = path.to_svg(&tolerance, true);
        assert_eq!(path.to_string().as_str(), "M5 5v10h10V5H5Z");
    }

    #[test]
    fn open_segment_ending_where_the_next_starts() {
        // The first segment ends at (6, 6), where the second starts, but it
        // started at (2, 2), so it's still open.
        let source = "M2 2l4 4M6 6L10 2";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);

        assert_eq!(
            path,
            Path(vec![
                Segment {
                    start: Point::splat(2.0),
                    data: vec![Data::LineTo(Point::splat(6.0))],
                    closed: false
                },
                Segment {
                    start: Point::splat(6.0),
                    data: vec![Data::LineTo(Point::new(10.0, 2.0))],
                    closed: false
                }
            ])
        );

        let path = path.to_svg(&tolerance, true);
        assert_eq!(path.to_string().as_str(), "m2 2 4 4m0 0 4-4");
    }

    #[test]
    fn closed_segment_followed_by_another() {
        // The first segment returns to its own start, so it's closed the same
        // as it would be were it the last.
        let source = "M5,5 L5,15 L15,15 L15,5 L5,5 M20,20 L25,25";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);

        assert_eq!(
            path.0.iter().map(Segment::closed).collect::<Vec<_>>(),
            vec![true, false]
        );

        let path = path.to_svg(&tolerance, true);
        assert_eq!(path.to_string().as_str(), "M5 5v10h10V5H5Zm15 15 5 5");
    }

    #[test]
    fn closed_segment_of_a_single_point() {
        // `M8 8z` has no data, but it's still closed: with round or square
        // caps it's drawn as a dot.
        let source = "M2 2h4M8 8z";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);

        assert_eq!(
            path.0.iter().map(Segment::closed).collect::<Vec<_>>(),
            vec![false, true]
        );

        let path = path.to_svg(&tolerance, true);
        assert_eq!(path.to_string().as_str(), "M2 2h4M8 8Z");
    }

    #[test]
    fn close_after_close() {
        // A second `Z` right after the first closes nothing.
        let source = "M4 4v4zz";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);

        assert_eq!(
            path.0.iter().map(Segment::closed).collect::<Vec<_>>(),
            vec![true]
        );

        let path = path.to_svg(&tolerance, true);
        assert_eq!(path.to_string().as_str(), "M4 4v4Z");
    }

    #[test]
    fn move_after_close() {
        // The move after `Z` starts the next segment, not one of its own,
        // and a move at the end starts one without data
        for (source, starts, closed) in [
            (
                "M1 1h4v4zM5 5h1",
                vec![Point::splat(1.0), Point::splat(5.0)],
                vec![true, false],
            ),
            (
                "M1 1h4v4zm5 5h1",
                vec![Point::splat(1.0), Point::splat(6.0)],
                vec![true, false],
            ),
            (
                "M1 1h4v4zM5 5",
                vec![Point::splat(1.0), Point::splat(5.0)],
                vec![true, false],
            ),
            (
                "M1 1h4M5 5",
                vec![Point::splat(1.0), Point::splat(5.0)],
                vec![false, false],
            ),
            (
                "M1 1M5 5",
                vec![Point::splat(1.0), Point::splat(5.0)],
                vec![false, false],
            ),
            ("M5 5", vec![Point::splat(5.0)], vec![false]),
        ] {
            let path = crate::Path::parse_string(source).unwrap();
            let tolerance = Tolerance::default();
            let path = Path::from_svg(&path, &tolerance);

            assert_eq!(
                path.0.iter().map(|s| *s.start()).collect::<Vec<_>>(),
                starts,
                "{source}"
            );
            assert_eq!(
                path.0.iter().map(Segment::closed).collect::<Vec<_>>(),
                closed,
                "{source}"
            );
        }
    }

    #[test]
    fn cubic_curves() {
        let source = "m10 90c20 0 15-80 40-80s20 80 40 80";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);
        let path = path.to_svg(&tolerance, true);
        assert_eq!(
            path.to_string().as_str(),
            "M10 90c20 0 15-80 40-80s20 80 40 80"
        );
    }

    #[test]
    fn move_relative_when_shorter() {
        // `m` where it's shorter than `M`, and `M` when both are as long
        for (source, expected) in [
            (
                "M100 100h10v10zM101 101c1 1 2 2 3 3",
                "M100 100h10v10Zm1 1c1 1 2 2 3 3",
            ),
            (
                "M100 100h10M111 101c1 1 2 2 3 3",
                "M100 100h10m1 1c1 1 2 2 3 3",
            ),
            ("M1 1h1v1zM3 3c1 1 2 2 3 3", "M1 1h1v1ZM3 3c1 1 2 2 3 3"),
        ] {
            let path = crate::Path::parse_string(source).unwrap();
            let tolerance = Tolerance::default();
            let mut path = Path::from_svg(&path, &tolerance);
            path.simplify(Options::RemoveEmptySegments, &tolerance);
            let path = path.to_svg(&tolerance, true);
            assert_eq!(path.to_string(), expected, "{source}");
        }

        let source = "M100 100h10M111 101M120 120h1";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance).to_svg(&tolerance, true);
        assert_eq!(path.to_string(), "M100 100h10m1 1m9 19h1");
    }

    #[test]
    fn relative_commands_go_where_absolute_ones_go() {
        use std::fmt::Write as _;

        // Each relative command goes from where the reader is, so rounding doesn't
        // add up: every end point is read within half a step of where it was
        let tolerance = Tolerance::default();
        let half_step = 0.5 / tolerance.precision().0;
        let mut source = String::new();
        for i in 0..50 {
            let x = f64::from(i) * 2.000_6;
            let shape = match i % 4 {
                0 => "h1v1z",
                1 => "l1.0004 1.0004l1.0004-2.0004",
                2 => "h1.0004v1.0004h.0004v-1.0004",
                _ => "c.0004 1.0004 1.0004 1.0004 1.0004.0004",
            };
            write!(source, "M{x} .0004{shape}").unwrap();
        }
        let path = crate::Path::parse_string(&source).unwrap();
        let mut path = Path::from_svg(&path, &tolerance);
        path.simplify(Options::RemoveEmptySegments, &tolerance);
        let written = path.to_svg(&tolerance, true);
        assert!(written.to_string().contains('m'), "{written}");

        let mut read = Path::from_svg(&written, &tolerance);
        read.simplify(Options::RemoveEmptySegments, &tolerance);
        assert_eq!(read.0.len(), path.0.len());
        for (before, after) in path.0.iter().zip(&read.0) {
            let points = |segment: &Segment| {
                std::iter::once(*segment.start())
                    .chain(segment.data().iter().map(Data::end_point))
                    .collect::<Vec<_>>()
            };
            for (a, b) in points(before).into_iter().zip(points(after)) {
                assert!(
                    (a.x - b.x).abs() <= half_step + 1e-9 && (a.y - b.y).abs() <= half_step + 1e-9,
                    "{a:?} read as {b:?} in {written}"
                );
            }
        }
    }

    #[test]
    fn arc_back_to_the_reader() {
        // An arc that ends where it starts draws nothing; ending one step on, it draws
        // nearly a circle, also when the reader's point is a sum like .1 + .2
        let source = "M.1 0h.2a5 5 0 1 1 .0001 0";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance).to_svg(&tolerance, true);
        assert_eq!(path.to_string(), "M.1 0h.2a5 5 0 1 1 .001 0");
    }

    #[test]
    fn small_arc_back_to_the_reader() {
        // Rounding the radii of an arc that ends one step on keeps it an arc
        let source = "M.44377 .9913L.38192 .50025a.10205 .10128 0 0 1-.0004.00023";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance).to_svg(&tolerance, true);
        assert!(!path.to_string().contains("a0 0"), "{path}");
    }

    #[test]
    fn quadratic_curves() {
        let source = "M10 50Q25 25 40 50t30 0 30 0 30 0 30 0 30 0";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);
        let path = path.to_svg(&tolerance, true);
        assert_eq!(
            path.to_string().as_str(),
            "M10 50q15-25 30 0t30 0 30 0 30 0 30 0 30 0"
        );
    }

    #[test]
    fn quadratic_curve_after_cubic() {
        // The quadratic's control point (-6, -15) reflects the cubic's last
        // one, but `T` reflects only a quadratic's: after a cubic it would
        // start at the current point and draw a straight line.
        let source = "M0 0S-6-6-6-10.5C-6-13.5-5-16.5-3-19.5";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);
        let path = path.to_svg(&tolerance, true);
        assert_eq!(path.to_string().as_str(), "M0 0s-6-6-6-10.5q0-4.5 3-9");
    }

    #[test]
    fn cubic_curve_after_quadratic() {
        // The cubic's first control point (30, -10) reflects the quadratic's,
        // but `S` reflects only a cubic's: after a quadratic it would start
        // at the current point.
        let source = "M0 0Q10 10 20 0C30-10 45-5 50 0";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);
        let path = path.to_svg(&tolerance, true);
        assert_eq!(path.to_string().as_str(), "M0 0q10 10 20 0c10-10 25-5 30 0");
    }

    /// The path written once and then written again from what was written.
    fn written_twice(source: &str) -> (String, String) {
        let tolerance = Tolerance::default();
        let write = |source: &str| {
            let path = crate::Path::parse_string(source).unwrap();
            Path::from_svg(&path, &tolerance)
                .to_svg(&tolerance, true)
                .to_string()
        };
        let once = write(source);
        let twice = write(&once);
        (once, twice)
    }

    #[test]
    fn smooth_curve_after_other_kind_reads_back() {
        // `S` after a quadratic and `T` after a cubic start at the current
        // point when read as well, so a second pass writes the same.
        for source in [
            "M0 0Q10 0 3 0C3 0 4 8-9 8",
            "M0 0Q10 10 20 0T40 0C40 0 50 10 60 0",
            "M0 0C0-5 5-5 5 0Q5 0 10 0",
        ] {
            let (once, twice) = written_twice(source);
            assert_eq!(once, twice, "{source}");
        }
        assert_eq!(
            written_twice("M0 0Q10 0 3 0C3 0 4 8-9 8").0,
            "M0 0q10 0 3 0S4 8-9 8"
        );
    }

    #[test]
    fn smooth_curve_starting_a_subpath() {
        // A subpath's first curve reflects no control point of the one before.
        assert_eq!(
            written_twice("M0 0Q7-4 3-7ZQ-7 4 9-5T3 10").0,
            "M0 0q7-4 3-7ZM0 0q-7 4 9-5T3 10"
        );
    }

    #[test]
    fn zero_length_line() {
        // A line of length zero stays a line: `Z` would move the current point
        // back to the subpath's start and shift everything after it.
        for (source, expected) in [
            ("M0 0L8 10L8 10Q0-2 10 7", "m0 0 8 10v0Q0-2 10 7"),
            ("M0 0L5 5L5 5", "m0 0 5 5v0"),
            ("M0 0L0 0L5 5", "M0 0v0l5 5"),
            // As the last command of a closed segment, it is that segment's `Z`.
            ("M0 0L5 0L5 5L5 5Z", "M0 0h5v5Z"),
        ] {
            let (once, twice) = written_twice(source);
            assert_eq!(once, expected, "{source}");
            assert_eq!(once, twice, "{source}");
        }
    }

    #[test]
    fn arc() {
        let source = "M6 10A6 4 10 1 0 14 10";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let path = Path::from_svg(&path, &tolerance);
        let path = path.to_svg(&tolerance, true);
        assert_eq!(path.to_string().as_str(), "M6 10a6 4 10 1 0 8 0");
    }

    #[test]
    fn curve_to_arc() {
        let source = "M0 0L0 0c2.761 0 5 2.239 5 5";
        let path = crate::Path::parse_string(source).unwrap();
        let tolerance = Tolerance::default();
        let mut path = Path::from_svg(&path, &tolerance);
        path.simplify(Options::all(), &tolerance);
        let path = path.to_svg(&tolerance, true);
        assert_eq!(path.to_string().as_str(), "M0 0a5 5 0 0 1 5 5");
    }
}
