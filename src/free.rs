use flat_projection::FlatPoint;
use log::{info, trace};
use ord_subset::OrdVar;
use std::collections::HashSet;

use crate::flat::to_flat_points;
use crate::parallel::*;
use crate::point::Point;
use crate::utils::Bound;
use crate::vincenty::vincenty_distance;

pub type Path = Vec<usize>;

#[derive(Debug)]
pub struct OptimizationResult {
    pub path: Path,
    pub distance: f32,
}

struct CacheItem {
    start: usize,
    stops: Vec<usize>,
    stop_set: HashSet<usize>,
    distance: f32,
}

pub fn optimize<T: Point>(route: &[T], break_at: f32, legs: usize) -> Option<OptimizationResult> {
    let mut iterations = 1;
    let flat_points = to_flat_points(route);

    info!("Calculating distance matrix (finish -> start)");
    let dist_matrix = half_dist_matrix(&flat_points);

    info!("Calculating solution graph");
    let graph = Graph::from_distance_matrix(&dist_matrix, legs);
    let mut best_valid = graph.find_best_valid_solution(route);

    info!(
        "-- New best solution: {:.3} km -> {:?}, {:?}",
        calculate_distance(route, &best_valid.path),
        best_valid.path,
        best_valid.distance,
    );
    let mut start_candidates: Vec<_> = graph.g[legs - 1]
        .iter()
        .enumerate()
        .filter(|(_, cell)| cell.distance > best_valid.distance)
        .map(|(start_index, cell)| StartCandidate {
            distance: cell.distance,
            start_index,
        })
        .collect();
    start_candidates.sort_by_key(|it| OrdVar::new_checked(it.distance));
    info!(
        "{} potentially better start points found",
        start_candidates.len()
    );

    // return early
    if start_candidates.is_empty() {
        let distance = calculate_distance(route, &best_valid.path);
        info!("Solution: {:?} ({:.3} km)", best_valid.path, distance);
        info!("{} iterations needed", iterations);
        return Some(OptimizationResult {
            distance,
            path: best_valid.path,
        });
    }

    let start_window = Bound {
        start: start_candidates
            .iter()
            .map(|c| c.start_index)
            .min()
            .unwrap(),
        stop: start_candidates
            .iter()
            .map(|c| c.start_index)
            .max()
            .unwrap(),
    };
    info!("Slide window is : {:?}", start_window);
    if let Some(improved) = best_valid.slide(route, &start_window, &flat_points, legs) {
        if improved.distance > best_valid.distance {
            best_valid = improved;
            info!(
                "-- New improved best solution: {:.3} km -> {:?}",
                calculate_distance(route, &best_valid.path),
                best_valid.path
            );
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
            info!(
                "-- New improved best solution: {:.3} km -> {:?}",
                calculate_distance(route, &best_valid.path),
                best_valid.path
            );
        }
    }

    let minimum_stop_index = find_minimum_stop_index(&dist_matrix, best_valid.distance);
    info!("Minimum stop index: {}", minimum_stop_index);

    let mut cache: Vec<CacheItem> = Vec::new();

    start_candidates.retain(|c| c.distance > best_valid.distance);
    info!(
        "{} potentially better start points left",
        start_candidates.len()
    );

    while let Some(candidate) = start_candidates.pop() {
        info!(
            "Checking candidate with start index: {}",
            candidate.start_index
        );
        let stops: Vec<usize> = candidate
            .get_valid_end_points(route)
            .into_iter()
            .filter(|x| x > &minimum_stop_index)
            .collect();
        if stops.is_empty() {
            continue;
        }
        let stop_set: HashSet<usize> = stops.iter().cloned().collect();
        let (use_caching, distance) = check_cache(
            &cache,
            &flat_points,
            &candidate,
            best_valid.distance,
            &stop_set,
        );
        if use_caching {
            let cache_item = CacheItem {
                start: candidate.start_index,
                stops,
                stop_set,
                distance,
            };
            cache.push(cache_item);
            info!("Found item in cache, continuing");
            continue;
        }

        // We can not score above maximum anymore
        if candidate.distance < break_at {
            return Some(best_valid);
        }
        iterations += 1;
        let finish_altitude = route[candidate.start_index].altitude();

        info!(
            "Calculating solution graph with start point at index {}",
            candidate.start_index
        );
        let candidate_graph = Graph::for_start_index(finish_altitude, &dist_matrix, route, legs);
        let best_valid_for_candidate = candidate_graph.find_best_valid_solution(route);
        let cache_item = CacheItem {
            start: candidate.start_index,
            stops,
            stop_set,
            distance: best_valid_for_candidate.distance,
        };
        cache.push(cache_item);

        if best_valid_for_candidate.distance > best_valid.distance {
            best_valid = best_valid_for_candidate;
            info!(
                "-- New best solution: {:.3} km -> {:?}",
                calculate_distance(route, &best_valid.path),
                best_valid.path
            );

            start_candidates.retain(|it| it.distance > best_valid.distance);
        } else {
            info!(
                "Discarding solution with {:.3} km -> {:?}",
                calculate_distance(route, &best_valid_for_candidate.path),
                &best_valid_for_candidate.path
            );
        }
        info!(
            "{} potentially better start points left",
            start_candidates.len()
        );
    }

    let distance = calculate_distance(route, &best_valid.path);
    info!("Solution: {:?} ({:.3} km)", best_valid.path, distance);

    info!("{} iterations needed", iterations);
    Some(OptimizationResult {
        distance,
        path: best_valid.path,
    })
}

#[derive(Debug)]
struct StartCandidate {
    distance: f32,
    start_index: usize,
}

impl StartCandidate {
    fn get_valid_end_points<T: Point>(&self, route: &[T]) -> Vec<usize> {
        let start_altitude = route[self.start_index].altitude();
        route
            .iter()
            .enumerate()
            .skip(self.start_index)
            .filter_map(|(index, cell)| {
                if start_altitude - cell.altitude() <= 1000 {
                    Some(index)
                } else {
                    None
                }
            })
            .collect()
    }
}

#[derive(Debug)]
struct Slide {
    start_index: usize,
    stop_index: usize,
    distance: f32,
}

impl OptimizationResult {
    fn slide<T: Point>(
        &self,
        route: &[T],
        start_window: &Bound,
        flat_points: &[FlatPoint<f32>],
        legs: usize,
    ) -> Option<OptimizationResult> {
        // Hold inner parts of path constant and adjust (wiggle) first and last to optimum
        // The result is not optimal, but comes in many cases very close, so it is a good starting point
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
                info!("New slide distance: {}", distance);
                Some(OptimizationResult {
                    path: new_path,
                    distance,
                })
            }
            None => None,
        }
    }
}

fn find_minimum_stop_index(dist_matrix: &[Vec<f32>], distance: f32) -> usize {
    // Find the minimum index where the resulting path needs to end to achieve a better result than distance
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

// Make use of a heuristic to quickly determine if end points will lead to a better result.
// Stop set are valid end points that could give a better result than best_distance for a current start candidate.
// If the stop set of the cached items is a subset of the current stop set, we make a rough guess for the maximum
// distance possible with the current stop set: Add the distane between the start point of the cached item
// the current item (offset_start) and the maxium distance between the end points of the cached item and the end
// points of the current item. If the resulting distane is smaller than current best_distance, we know that we can
// disregard the current candidate.
fn check_cache(
    cache: &[CacheItem],
    flat_points: &[FlatPoint<f32>],
    candidate: &StartCandidate,
    best_distance: f32,
    stop_set: &HashSet<usize>,
) -> (bool, f32) {
    let mut distance = 0.0;
    let mut use_caching: bool = false;
    for cache_item in cache.iter().rev() {
        let offset_start =
            flat_points[cache_item.start].distance(&flat_points[candidate.start_index]);
        if cache_item.distance + offset_start < best_distance {
            // Looking promising, we need superset, otherwise we can not do nothing
            // The reason subset is not enough is that a subset of allowed end indices may actually allow for
            // a better result as it is no longer 'hidden' by an end index which 'hides' it when going dynamic.
            if stop_set.is_superset(&cache_item.stop_set) {
                use_caching = true;
                distance = cache_item.distance;
                for to_check in stop_set.difference(&cache_item.stop_set) {
                    let offset_end = flat_points[*cache_item.stops.last().unwrap()]
                        .distance(&flat_points[*to_check]);
                    distance = offset_end + offset_start + cache_item.distance;
                    if distance > best_distance {
                        use_caching = false;
                        break;
                    }
                }
                if use_caching {
                    break;
                }
            }
        }
    }
    (use_caching, distance)
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

struct Graph {
    g: Vec<Vec<GraphCell>>,
}

#[derive(Debug)]
struct GraphCell {
    prev_index: usize,
    distance: f32,
}

impl Graph {
    fn from_distance_matrix(dist_matrix: &[Vec<f32>], legs: usize) -> Self {
        let mut graph: Vec<Vec<GraphCell>> = Vec::with_capacity(legs);

        trace!("-- Analyzing leg #{}", 6);

        let layer: Vec<GraphCell> = opt_par_iter(dist_matrix)
            .enumerate()
            .map(|(tp_index, distances)| {
                distances
                    .iter()
                    .enumerate()
                    .map(|(start_index, &distance)| GraphCell {
                        prev_index: start_index + tp_index,
                        distance,
                    })
                    .max_by_key(|cell| OrdVar::new_checked(cell.distance))
                    .unwrap()
            })
            .collect();

        graph.push(layer);

        for layer_index in 1..legs {
            trace!("-- Analyzing leg #{}", legs - layer_index);
            let last_layer = &graph[layer_index - 1];

            let layer: Vec<GraphCell> = opt_par_iter(dist_matrix)
                .enumerate()
                .map(|(tp_index, distances)| {
                    distances
                        .iter()
                        .zip(last_layer.iter().skip(tp_index))
                        .enumerate()
                        .map(|(prev_index, (&leg_dist, last_layer_cell))| {
                            let distance = last_layer_cell.distance + leg_dist;
                            GraphCell {
                                prev_index: prev_index + tp_index,
                                distance,
                            }
                        })
                        .max_by_key(|cell| OrdVar::new_checked(cell.distance))
                        .unwrap()
                })
                .collect();

            graph.push(layer);
        }

        Graph { g: graph }
    }

    // build a layered graph for a fixed start point which can be traversed
    // to find the best solution for the given start point
    // penalize finish points that to not adhere to the 1000m altitude rule
    fn for_start_index<T: Point>(
        start_altitude: i16,
        dist_matrix: &[Vec<f32>],
        points: &[T],
        legs: usize,
    ) -> Self {
        let mut graph: Vec<Vec<GraphCell>> = Vec::with_capacity(legs);

        trace!("-- Analyzing leg #{}", 6);

        // Only use finish points which are compliant to 1000m rule
        //
        // assuming X is the first turnpoint, what is the distance to `start_index`?

        let layer: Vec<GraphCell> = opt_par_iter(dist_matrix)
            .enumerate()
            .map(|(tp_index, distances)| {
                distances
                    .iter()
                    .enumerate()
                    .map(|(finish_index, &distance)| {
                        let finish = &points[finish_index + tp_index];
                        let altitude_delta = start_altitude - finish.altitude();
                        if altitude_delta <= 1000 {
                            GraphCell {
                                prev_index: finish_index + tp_index,
                                distance,
                            }
                        } else {
                            GraphCell {
                                prev_index: finish_index + tp_index,
                                distance: distance - 100_000.0,
                            }
                        }
                    })
                    .max_by_key(|cell| OrdVar::new_checked(cell.distance))
                    .unwrap()
            })
            .collect();
        graph.push(layer);

        for layer_index in 1..legs {
            trace!("-- Analyzing leg #{}", legs - layer_index);

            // layer: 1 / leg: 2
            //
            // assuming X is the second turnpoint, what is the first turnpoint
            // that results in the highest total distance?
            //
            // layer: 2 / leg: 3
            //
            // assuming X is the third turnpoint, what is the second turnpoint
            // that results in the highest total distance?

            let last_layer = &graph[layer_index - 1];
            let layer: Vec<GraphCell> = opt_par_iter(dist_matrix)
                .enumerate()
                .map(|(tp_index, distances)| {
                    distances
                        .iter()
                        .zip(last_layer.iter().skip(tp_index))
                        .enumerate()
                        .map(|(prev_index, (&leg_dist, last_layer_cell))| {
                            let distance = last_layer_cell.distance + leg_dist;
                            GraphCell {
                                prev_index: prev_index + tp_index,
                                distance,
                            }
                        })
                        .max_by_key(|cell| OrdVar::new_checked(cell.distance))
                        .unwrap()
                })
                .collect();
            graph.push(layer);
        }

        Graph { g: graph }
    }

    // Finds the path in the graph which maximizes the distance between its elements.
    // Respect the 1000m altitude restriction.
    fn find_best_valid_solution<T: Point>(&self, points: &[T]) -> OptimizationResult {
        let last_graph_row = self.g.last().unwrap();

        let offset = points.len() - last_graph_row.len();

        last_graph_row
            .iter()
            .enumerate()
            .filter_map(|(index, cell)| {
                let iter = GraphIterator {
                    graph: self,
                    next: Some((self.g.len(), index + offset)),
                    offset,
                };

                let mut path = iter.collect::<Vec<_>>();
                if *path.first().unwrap() > *path.last().unwrap() {
                    path.reverse();
                }

                let start_index = *path.first().unwrap();
                let finish_index = *path.last().unwrap();
                let start = &points[start_index];
                let finish = &points[finish_index];
                let altitude_delta = start.altitude() - finish.altitude();
                if altitude_delta <= 1000 {
                    trace!(
                        "Start: {} -> Finish: {}: {} km",
                        start_index,
                        finish_index,
                        cell.distance
                    );
                    Some(OptimizationResult {
                        distance: cell.distance,
                        path,
                    })
                } else {
                    trace!(
                        "No: Start: {} -> Finish: {}: {} km",
                        start_index,
                        finish_index,
                        cell.distance
                    );
                    None
                }
            })
            .max_by_key(|result| OrdVar::new_checked(result.distance))
            .unwrap()
    }

    // Finds the path in the graph which maximizes the distance between its elements.
    // Do not respect the 1000m altitude restriction.
    fn find_best_solution<T: Point>(&self, points: &[T]) -> OptimizationResult {
        let last_graph_row = self.g.last().unwrap();

        let offset = points.len() - last_graph_row.len();

        last_graph_row
            .iter()
            .enumerate()
            .filter_map(|(index, cell)| {
                let iter = GraphIterator {
                    graph: self,
                    next: Some((self.g.len(), index + offset)),
                    offset,
                };

                let mut path = iter.collect::<Vec<_>>();
                if *path.first().unwrap() > *path.last().unwrap() {
                    path.reverse();
                }
                Some(OptimizationResult {
                    distance: cell.distance,
                    path,
                })
            })
            .max_by_key(|result| OrdVar::new_checked(result.distance))
            .unwrap()
    }
}

struct GraphIterator<'a> {
    graph: &'a Graph,
    next: Option<(usize, usize)>,
    offset: usize,
}

impl Iterator for GraphIterator<'_> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        self.next?;
        let (layer, index) = self.next.unwrap();
        self.next = if layer == 0 {
            None
        } else {
            let next_layer = layer - 1;
            let next_index = self.graph.g[next_layer][index - self.offset].prev_index;
            Some((next_layer, next_index))
        };

        Some(index)
    }
}

// Calculate the total distance of a solution
fn calculate_distance<T: Point>(points: &[T], path: &Path) -> f32 {
    path.iter()
        .zip(path.iter().skip(1))
        .map(|(i1, i2)| (&points[*i1], &points[*i2]))
        .map(|(fix1, fix2)| vincenty_distance(fix1, fix2))
        .sum()
}
