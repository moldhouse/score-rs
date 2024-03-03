use flat_projection::FlatPoint;
use std::collections::HashSet;

use crate::cache::{Cache, CacheItem};
use crate::flat::to_flat_points;
use crate::graph::Graph;
use crate::parallel::*;
use crate::point::Point;
use crate::result::{Bound, OptimizationResult};

// Find the optimal set of (legs + 1) turnpoints, such that the sum of the inter turnpoints distances is maximized.
// Break if no solution above break_at km an be found
pub fn optimize<T: Point>(route: &[T], break_at: f32, legs: usize) -> Option<OptimizationResult> {
    let flat_points = to_flat_points(route);
    let dist_matrix = half_dist_matrix(&flat_points);

    let graph = Graph::from_distance_matrix(&dist_matrix, legs);
    let mut best_valid = graph.find_best_valid_solution(route);

    let mut start_candidates = graph.get_start_candidates(best_valid.distance);
    if start_candidates.is_empty() {
        return Some(OptimizationResult::new(best_valid.path, route));
    }

    let start_window = Bound::from(start_candidates.as_ref());
    if let Some(improved) = best_valid.slide(route, &start_window, &flat_points, legs) {
        if improved.distance > best_valid.distance {
            best_valid = improved;
        }
    }

    // try sliding over solution obtained without alt limit for edge cases
    if let Some(improved) =
        graph
            .find_best_solution(route)
            .slide(route, &start_window, &flat_points, legs)
    {
        if improved.distance > best_valid.distance {
            best_valid = improved;
        }
    }

    let min_stop_idx = find_min_stop_idx(&dist_matrix, best_valid.distance);
    let mut cache = Cache::new();

    start_candidates.retain(|c| c.distance > best_valid.distance);

    while let Some(candidate) = start_candidates.pop() {
        let stops: Vec<usize> = candidate.get_valid_end_points(route, min_stop_idx);
        if stops.is_empty() {
            continue;
        }
        let stop_set: HashSet<usize> = stops.iter().cloned().collect();
        let (use_caching, distance) =
            cache.check(&flat_points, &candidate, best_valid.distance, &stop_set);
        if use_caching {
            let cache_item = CacheItem {
                start: candidate.start_index,
                stops,
                stop_set,
                distance,
            };
            cache.set(cache_item);
            continue;
        }

        // We can not score above maximum anymore
        if candidate.distance < break_at {
            return Some(best_valid);
        }
        let finish_altitude = route[candidate.start_index].altitude();

        let candidate_graph = Graph::for_start_index(finish_altitude, &dist_matrix, route, legs);
        let best_valid_for_candidate = candidate_graph.find_best_valid_solution(route);
        let cache_item = CacheItem {
            start: candidate.start_index,
            stops,
            stop_set,
            distance: best_valid_for_candidate.distance,
        };
        cache.set(cache_item);

        if best_valid_for_candidate.distance > best_valid.distance {
            best_valid = best_valid_for_candidate;
            start_candidates.retain(|it| it.distance > best_valid.distance);
        }
    }

    Some(OptimizationResult::new(best_valid.path, route))
}

// Calculate the cumulative distance when going from fix to fix. This places an upper limit on the
// distance achievable with n legs and is used to calculate a minimum index where a path needs to end
// to have the possibility to achieve a better result than distance
fn find_min_stop_idx(dist_matrix: &[Vec<f32>], distance: f32) -> usize {
    let mut i = 0;
    let mut sum = 0.0;
    loop {
        sum += dist_matrix[i][1];
        if sum > distance {
            return i;
        };
        i += 1;
    }
}

// Generate a triangular matrix with the distances in kilometers between all points.
// For each point, the distance to the following points is saved. This only allows
// calculation of backward min-marginals
pub fn half_dist_matrix(flat_points: &[FlatPoint<f32>]) -> Vec<Vec<f32>> {
    opt_par_iter(flat_points)
        .enumerate()
        .map(|(i, p1)| flat_points[i..].iter().map(|p2| p1.distance(p2)).collect())
        .collect()
}

#[cfg(test)]
mod tests {
    use crate::free;
    use crate::free::OptimizationResult;
    use crate::point::PointImpl;
    use assert_approx_eq::assert_approx_eq;
    use igc::records::BRecord;
    use igc::util::Time;

    const LEGS: usize = 6;

    #[test]
    fn free_distance() {
        let release = Time::from_hms(8, 12, 29);
        let result = run_free_test(include_str!("../fixtures/2023-06-17_288167.igc"), release);
        assert_approx_eq!(result.distance, 1018.5, 0.1);
        assert_eq!(result.path, [0, 936, 2847, 3879, 5048, 7050, 8128]);
    }

    #[test]
    fn free_distance_with_1000m() {
        let release = Time::from_hms(8, 16, 30);
        let result = run_free_test(include_str!("../fixtures/schunk_1000m.igc"), release);
        assert_approx_eq!(result.distance, 1158.61, 0.1);
        assert_eq!(result.path, [335, 10099, 14740, 15482, 24198, 34160, 35798]);
    }

    fn run_free_test(file: &str, release: Time) -> OptimizationResult {
        env_logger::try_init().ok();

        let fixes = file
            .lines()
            .filter(|l| l.starts_with('B'))
            .filter_map(|line| {
                BRecord::parse(&line).ok().map_or(None, |record| {
                    if record.timestamp.seconds_since_midnight() >= release.seconds_since_midnight()
                    {
                        Some(PointImpl {
                            latitude: record.pos.lat.into(),
                            longitude: record.pos.lon.into(),
                            altitude: record.pressure_alt,
                        })
                    } else {
                        None
                    }
                })
            })
            .collect::<Vec<_>>();

        free::optimize(&fixes, 0.0, LEGS).unwrap()
    }
}
