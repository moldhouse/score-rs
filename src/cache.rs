use flat_projection::FlatPoint;
use std::collections::HashSet;

use crate::graph::StartCandidate;

pub struct Cache {
    cache: Vec<CacheItem>,
}

pub struct CacheItem {
    pub start: usize,
    pub stops: Vec<usize>,
    pub stop_set: HashSet<usize>,
    pub distance: f32,
}

impl Cache {
    pub fn new() -> Cache {
        Cache { cache: Vec::new() }
    }
    pub fn set(&mut self, item: CacheItem) {
        self.cache.push(item);
    }

    // Make use of a heuristic to quickly determine if end points will lead to a better result.
    // Stop set are valid end points that could give a better result than best_distance for a current start candidate.
    // If the stop set of the cached items is a subset of the current stop set, we make a rough guess for the maximum
    // distance possible with the current stop set: Add the distane between the start point of the cached item
    // the current item (offset_start) and the maxium distance between the end points of the cached item and the end
    // points of the current item. If the resulting distane is smaller than current best_distance, we know that we can
    // disregard the current candidate.
    pub fn check(
        &self,
        flat_points: &[FlatPoint<f32>],
        candidate: &StartCandidate,
        best_distance: f32,
        stop_set: &HashSet<usize>,
    ) -> (bool, f32) {
        let mut distance = 0.0;
        let mut use_caching: bool = false;
        for cache_item in self.cache.iter().rev() {
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
}
