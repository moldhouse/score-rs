use crate::graph::StartCandidate;
use crate::point::{ApproxDistance, Path, Point, Valid, VincentyDistance};
use flat_projection::FlatPoint;
use ord_subset::OrdVar;

#[derive(Debug)]
pub struct OptimizationResult {
    pub path: Path,
    pub distance: f32,
}

impl OptimizationResult {
    pub fn new<T: Point>(path: Path, route: &[T]) -> Self {
        let distance = route.cum_distance(&path);
        OptimizationResult { path, distance }
    }
}

#[derive(Debug, PartialEq)]
pub struct Bound {
    pub start: usize,
    pub stop: usize,
}

impl From<&[StartCandidate]> for Bound {
    fn from(candidates: &[StartCandidate]) -> Self {
        Bound {
            start: candidates.iter().map(|c| c.start).min().unwrap(),
            stop: candidates.iter().map(|c| c.start).max().unwrap(),
        }
    }
}

#[derive(Debug)]
struct Slide {
    start: usize,
    stop: usize,
    distance: f32,
}

impl OptimizationResult {
    // Hold inner parts of path constant and adjust (wiggle) first and last to optimum
    // The result is not optimal, but comes in many cases very close, so it is a good starting point
    pub fn slide<T: Point>(
        &self,
        route: &[T],
        flat_points: &[FlatPoint<f32>],
        start_window: &Bound,
    ) -> Option<OptimizationResult> {
        let second = self.path[1];
        let penultimate = self.path[self.path.len() - 2];

        let slide = (start_window.start..start_window.stop.min(second))
            .filter_map(|start| {
                (penultimate..route.len())
                    .filter_map(|stop| {
                        if route.valid(start, stop) {
                            // all the other legs stay constant, so we can ignore them to find the max
                            let first_leg = flat_points.distance(start, second);
                            let last_leg = flat_points.distance(stop, penultimate);
                            Some(Slide {
                                start,
                                stop,
                                distance: first_leg + last_leg,
                            })
                        } else {
                            None
                        }
                    })
                    .max_by_key(|slide| OrdVar::new_checked(slide.distance))
            })
            .max_by_key(|slide| OrdVar::new_checked(slide.distance));

        match slide {
            Some(slide) => {
                let mut path = self.path.clone();
                path[0] = slide.start;
                path[self.path.len() - 1] = slide.stop;
                let distance = flat_points.cum_distance(&path);
                Some(OptimizationResult { path, distance })
            }
            None => None,
        }
    }
}
