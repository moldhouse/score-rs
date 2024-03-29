use flat_projection::FlatPoint;

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
    if let Some(improved) = best_valid.optimize_by_sliding(route, &flat_points, &start_window) {
        if improved.distance > best_valid.distance {
            best_valid = improved;
        }
    }

    // for edge cases, sliding over the best invalid solution produces a valid one
    let best_invalid = graph.find_best_solution(route);
    if let Some(improved) = best_invalid.optimize_by_sliding(route, &flat_points, &start_window) {
        if improved.distance > best_valid.distance {
            best_valid = improved;
        }
    }

    let minimum_stop = find_minimum_stop(&dist_matrix, best_valid.distance);
    let mut cache = Cache::new();

    start_candidates.retain(|c| c.distance > best_valid.distance);

    while let Some(candidate) = start_candidates.pop() {
        if candidate.distance < break_at {
            return Some(best_valid);
        }
        let stops = candidate.get_valid_stops(route, minimum_stop);
        if stops.is_empty() {
            continue;
        }
        let mut to_check = CacheItem::from_candidate(&candidate, stops);
        if cache.check(&mut to_check, &flat_points, best_valid.distance) {
            // there is no need to add this to the cache, because the relation is transitive
            // if A provides an upperbound for B, and B provides an upperbound for a later C
            // then A provides an upperbound for C, so we don't need to add B to the cache
            //
            // BUT: adding this to the cache provides a speed-up on the test suite
            cache.set(to_check);
            continue;
        }

        // do the full (expensive) optimization
        let candidate_graph = Graph::for_candidate(&candidate, &dist_matrix, route, legs);
        let best_valid_for_candidate = candidate_graph.find_best_valid_solution(route);

        to_check.distance = best_valid_for_candidate.distance;
        cache.set(to_check);

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
fn find_minimum_stop(dist_matrix: &[Vec<f32>], distance: f32) -> usize {
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
