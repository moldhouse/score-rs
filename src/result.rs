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
struct SlidingResult {
    start: usize,
    stop: usize,
    distance: f32,
}

impl OptimizationResult {
    // Hold inner parts of path constant and adjust (wiggle) first and last to optimum
    // The result is not optimal, but comes in many cases very close, so it is a good starting point
    pub fn optimize_by_sliding<T: Point>(
        &self,
        route: &[T],
        flat_points: &[FlatPoint<f32>],
        start_window: &Bound,
    ) -> Option<OptimizationResult> {
        if self.path.len() < 3 {
            return None;
        }
        // adjusting makes only sense to the following / previous turnpoint
        let second = self.path[1];
        let first_leg_start = start_window.start..start_window.stop.min(second);

        let penultimate = self.path[self.path.len() - 2];
        let last_leg_start = penultimate..route.len();

        let sliding_result = first_leg_start
            .filter_map(|start| {
                last_leg_start
                    .clone()
                    .filter(|stop| route.valid(start, *stop))
                    .map(|stop| {
                        // all the other legs stay constant, so we ignore them to find the max
                        let first_leg = flat_points.distance(start, second);
                        let last_leg = flat_points.distance(stop, penultimate);
                        SlidingResult {
                            start,
                            stop,
                            distance: first_leg + last_leg,
                        }
                    })
                    .max_by_key(|slide| OrdVar::new_checked(slide.distance))
            })
            .max_by_key(|slide| OrdVar::new_checked(slide.distance));

        sliding_result.map(|slide| self.from_slide_result(route, slide))
    }

    // create a new OptimizationResult after the sliding optimization
    fn from_slide_result<T: Point>(&self, route: &[T], slide: SlidingResult) -> Self {
        let mut path = self.path.clone();
        path[0] = slide.start;
        path[self.path.len() - 1] = slide.stop;
        let distance = route.cum_distance(&path);
        OptimizationResult { path, distance }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::point::PointImpl;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn bound_finds_min_and_max() {
        let candidates = vec![
            StartCandidate {
                start: 100,
                distance: 0.0,
            },
            StartCandidate {
                start: 20,
                distance: 0.0,
            },
        ];
        let bound = Bound::from(candidates.as_slice());
        assert_eq!(bound.start, 20);
        assert_eq!(bound.stop, 100);
    }

    #[test]
    fn from_slide_result_updates_path() {
        let route = vec![
            PointImpl {
                latitude: 0.0,
                longitude: 0.0,
                altitude: 0,
            },
            PointImpl {
                latitude: 1.0,
                longitude: 1.0,
                altitude: 0,
            },
        ];
        let result = OptimizationResult {
            path: vec![1, 0, 0, 1],
            distance: 0.0,
        };
        let slide = SlidingResult {
            start: 0,
            stop: 0,
            distance: 100.0,
        };
        let improved = result.from_slide_result(&route, slide);
        assert_eq!(improved.path, vec![0, 0, 0, 0]);
    }

    #[test]
    fn from_slide_result_updates_distance() {
        let route = vec![
            PointImpl {
                latitude: 0.0,
                longitude: 0.0,
                altitude: 0,
            },
            PointImpl {
                latitude: 10.0,
                longitude: 10.0,
                altitude: 0,
            },
        ];
        let result = OptimizationResult {
            path: vec![0, 1, 0, 0],
            distance: 100.0,
        };
        let slide = SlidingResult {
            start: 0,
            stop: 0,
            distance: 100.0,
        };
        let improved = result.from_slide_result(&route, slide);
        assert_approx_eq!(improved.distance, 3130.22, 0.01);
    }

    #[test]
    fn optimize_by_sliding_produces_better_result() {
        // TODO this test case show that optimize_by_sliding is too compley and should be simplified
        let route = vec![
            PointImpl {
                latitude: 0.0,
                longitude: 0.0,
                altitude: 0,
            };
            5
        ];
        let result = OptimizationResult {
            path: vec![1, 1, 1],
            distance: 100.0,
        };
        let flat_points = vec![
            FlatPoint { x: 0.0, y: 0.0 },
            FlatPoint { x: 1.0, y: 1.0 },
            FlatPoint { x: 2.0, y: 2.0 },
            FlatPoint { x: 3.0, y: 3.0 },
            FlatPoint { x: 4.0, y: 4.0 },
        ];
        let start_window = Bound { start: 0, stop: 5 };
        let improved = result.optimize_by_sliding(&route, &flat_points, &start_window);
        assert_eq!(improved.unwrap().path, vec![0, 1, 4]);
    }
}
