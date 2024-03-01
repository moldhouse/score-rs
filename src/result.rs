use crate::graph::StartCandidate;
use crate::point::Point;
use crate::vincenty::vincenty_distance;
use flat_projection::FlatPoint;
use ord_subset::OrdVar;

pub type Path = Vec<usize>;

fn calculate_distance<T: Point>(points: &[T], path: &Path) -> f32 {
    path.iter()
        .zip(path.iter().skip(1))
        .map(|(i1, i2)| (&points[*i1], &points[*i2]))
        .map(|(fix1, fix2)| vincenty_distance(fix1, fix2))
        .sum()
}
#[derive(Debug)]
pub struct OptimizationResult {
    pub path: Path,
    pub distance: f32,
}

impl OptimizationResult {
    pub fn new<T: Point>(path: Path, points: &[T]) -> Self {
        let distance = calculate_distance(points, &path);
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
            start: candidates.iter().map(|c| c.start_index).min().unwrap(),
            stop: candidates.iter().map(|c| c.start_index).max().unwrap(),
        }
    }
}

#[derive(Debug)]
struct Slide {
    start_index: usize,
    stop_index: usize,
    distance: f32,
}

impl OptimizationResult {
    // Hold inner parts of path constant and adjust (wiggle) first and last to optimum
    // The result is not optimal, but comes in many cases very close, so it is a good starting point
    pub fn slide<T: Point>(
        &self,
        route: &[T],
        start_window: &Bound,
        flat_points: &[FlatPoint<f32>],
        legs: usize,
    ) -> Option<OptimizationResult> {
        let slide = (start_window.start..start_window.stop.min(self.path[1]))
            .filter_map(|start_index| {
                (self.path[legs - 1]..route.len())
                    .filter_map(|stop_index| {
                        if route[start_index].altitude() - route[stop_index].altitude() <= 1000 {
                            let distance = flat_points[start_index]
                                .distance(&flat_points[self.path[1]])
                                + flat_points[stop_index]
                                    .distance(&flat_points[self.path[legs - 1]]);
                            Some(Slide {
                                start_index,
                                stop_index,
                                distance,
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
                let mut new_path = self.path.clone();
                new_path[0] = slide.start_index;
                new_path[legs] = slide.stop_index;
                let distance = new_path
                    .iter()
                    .zip(new_path.iter().skip(1))
                    .map(|(i1, i2)| flat_points[*i1].distance(&flat_points[*i2]))
                    .sum();
                Some(OptimizationResult {
                    path: new_path,
                    distance,
                })
            }
            None => None,
        }
    }
}
