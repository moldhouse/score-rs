use ord_subset::OrdVar;

use crate::parallel::*;
use crate::point::{Point, Valid};
use std::collections::HashSet;

use crate::result::OptimizationResult;

#[derive(Debug)]
pub struct StartCandidate {
    pub distance: f32,
    pub start: usize,
}

impl StartCandidate {
    pub fn new(distance: f32, start: usize) -> Self {
        StartCandidate { distance, start }
    }

    // Return all points that would be valid endpoints for a route with the StartCandidate
    // Also filter out endpoints that are below minimum_stop, as they can not beat the current best
    pub fn get_valid_stops<T: Point>(&self, route: &[T], minimum_stop: usize) -> HashSet<usize> {
        let start_altitude = route[self.start].altitude();
        route
            .iter()
            .enumerate()
            .skip(self.start)
            .filter(|(index, cell)| {
                *index > minimum_stop && start_altitude - cell.altitude() <= 1000
            })
            .map(|(index, _)| index)
            .collect()
    }
}

pub struct Graph {
    g: Vec<Vec<GraphCell>>,
}

#[derive(Debug)]
struct GraphCell {
    prev_index: usize,
    distance: f32,
}

// A layered graph with size [n_turnpoints, n_gps_points]
//
// Cell at [i, j]: If GPS point j is selected as turnpoint number i, what is the distance I can achieve via the previous i-1 turnpoints?
// By selecting the maximum distance cell in the last layer, the graph can be iterated to find the best path.
impl Graph {
    // Return the remaining candidates that have the option of being better than the current best
    pub fn get_start_candidates(&self, current_best: f32) -> Vec<StartCandidate> {
        let mut candidates: Vec<_> = self
            .g
            .last()
            .unwrap()
            .iter()
            .enumerate()
            .filter(|(_, cell)| cell.distance > current_best)
            .map(|(start, cell)| StartCandidate::new(cell.distance, start))
            .collect();
        candidates.sort_by_key(|it| OrdVar::new_checked(it.distance));
        candidates
    }

    // Build the graph without considering the 1000m rule
    pub fn from_distance_matrix(dist_matrix: &[Vec<f32>], legs: usize) -> Self {
        let mut graph: Vec<Vec<GraphCell>> = Vec::with_capacity(legs);
        let layer: Vec<GraphCell> = opt_par_iter(dist_matrix)
            .enumerate()
            .map(|(tp_index, distances)| {
                distances
                    .iter()
                    .enumerate()
                    .map(|(start, &distance)| GraphCell {
                        prev_index: start + tp_index,
                        distance,
                    })
                    .max_by_key(|cell| OrdVar::new_checked(cell.distance))
                    .unwrap()
            })
            .collect();

        graph.push(layer);

        for layer_index in 1..legs {
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

    // Build a layered graph for a fixed start point which can be traversed
    // to find the best solution for the given start point.
    // Penalize finish points that to not adhere to the 1000m altitude rule
    pub fn for_candidate<T: Point>(
        candidate: &StartCandidate,
        dist_matrix: &[Vec<f32>],
        route: &[T],
        legs: usize,
    ) -> Self {
        let mut graph: Vec<Vec<GraphCell>> = Vec::with_capacity(legs);

        let layer: Vec<GraphCell> = opt_par_iter(dist_matrix)
            .enumerate()
            .map(|(tp_index, distances)| {
                distances
                    .iter()
                    .enumerate()
                    .map(|(finish_index, &distance)| {
                        if route.valid(candidate.start, finish_index + tp_index) {
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

    // Iterate the graph to find the path which maximizes the distance between its elements.
    // Respect the 1000m altitude restriction.
    //
    // Note: This function does not guarantee optimality. For every endpoint, it will find the best path without
    // the altitude constraint and disregard the ones that do not satisfy the constraint. This is not equivalent to
    // finding the best path that satisfies the constraint for every endpoint.
    // The result of this function can be used as a lower bound for a more complex optimization algorithm.
    //
    // If the graph has been build using Graph::for_start, the result ensures optimality for the given start point.
    pub fn find_best_valid_solution<T: Point>(&self, route: &[T]) -> OptimizationResult {
        let last_graph_row = self.g.last().unwrap();
        let offset = route.len() - last_graph_row.len();

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
                if route.valid(path[0], path[path.len() - 1]) {
                    Some(OptimizationResult {
                        distance: cell.distance,
                        path,
                    })
                } else {
                    None
                }
            })
            .max_by_key(|result| OrdVar::new_checked(result.distance))
            .unwrap()
    }

    // Iterate the graph to find the path which maximizes the distance between its elements.
    // Respect the 1000m altitude restriction.
    pub fn find_best_solution<T: Point>(&self, route: &[T]) -> OptimizationResult {
        let last_graph_row = self.g.last().unwrap();
        let offset = route.len() - last_graph_row.len();

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
