use log::trace;
use ord_subset::OrdVar;

use crate::parallel::*;
use crate::point::Point;

pub type Path = Vec<usize>;

#[derive(Debug)]
pub struct OptimizationResult {
    pub path: Path,
    pub distance: f32,
}

#[derive(Debug)]
pub struct StartCandidate {
    pub distance: f32,
    pub start_index: usize,
}

impl StartCandidate {
    pub fn get_valid_end_points<T: Point>(&self, route: &[T]) -> Vec<usize> {
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

pub struct Graph {
    g: Vec<Vec<GraphCell>>,
}

#[derive(Debug)]
struct GraphCell {
    prev_index: usize,
    distance: f32,
}

impl Graph {
    pub fn get_start_candidates(&self, current_best: f32) -> Vec<StartCandidate> {
        let mut candidates: Vec<_> = self
            .g
            .last()
            .unwrap()
            .iter()
            .enumerate()
            .filter(|(_, cell)| cell.distance > current_best)
            .map(|(start_index, cell)| StartCandidate {
                distance: cell.distance,
                start_index,
            })
            .collect();
        candidates.sort_by_key(|it| OrdVar::new_checked(it.distance));
        candidates
    }

    pub fn from_distance_matrix(dist_matrix: &[Vec<f32>], legs: usize) -> Self {
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
    pub fn for_start_index<T: Point>(
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
    pub fn find_best_valid_solution<T: Point>(&self, points: &[T]) -> OptimizationResult {
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
    pub fn find_best_solution<T: Point>(&self, points: &[T]) -> OptimizationResult {
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
