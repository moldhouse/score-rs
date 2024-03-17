// Building the Graph from the distance matrix for each start point is computationally expensive.
// The cache stores the results of previous graph build and tries to understand if any item in the
// cache which is sufficiently similiar to the current candidate can be used to place an upper bound
// on the maximum achievable distance with this candidate.

use crate::graph::StartCandidate;
use crate::point::ApproxDistance;
use flat_projection::FlatPoint;
use std::collections::HashSet;

pub struct CacheItem {
    pub start: usize,
    pub stops: HashSet<usize>,
    pub max_stop: usize,
    pub distance: f32,
}

impl CacheItem {
    pub fn from_candidate(candidate: &StartCandidate, stops: HashSet<usize>) -> CacheItem {
        let max_stop = *stops.iter().max().unwrap();
        CacheItem {
            start: candidate.start,
            stops,
            max_stop,
            distance: 0.0,
        }
    }
    // If the current stop set of an incoming item is super set of the stop set of the cached item,
    // we can place an upper bound on the possible distance with the current stop set:
    //
    // The sum of:
    // 1. Offset start: The distance between the two start candidates
    // 2. The max distance of the cached item
    // 3. The max distance of any stop in the current stop set (that is not in the cached stop set)
    //    to the last (or any single other) item in the cached stop set (the last leads to the lowest bound in the most cases)
    // If this upper bound is lower than the current best distance, we can rule out the candidate.
    //
    // Note: If the possible endpoints of the incoming item would be a subset of the cached item, the maximum altitude of the
    // stop set could be higher, therefore allowing for more start points then the cached item. In this case, the cached item
    // can not be used to calculate an upper bound.
    pub fn places_upperbound(
        &self,
        candidate: &mut CacheItem,
        flat_points: &[FlatPoint<f32>],
        best_distance: f32,
    ) -> bool {
        let start_offset = flat_points.distance(self.start, candidate.start);
        candidate.distance = self.distance + start_offset;
        if candidate.distance >= best_distance {
            // this item does not provide an upper bound below best_distance
            return false;
        }
        if !candidate.stops.is_superset(&self.stops) {
            return false;
        }
        for to_check in candidate.stops.difference(&self.stops) {
            let stop_offset = flat_points.distance(self.max_stop, *to_check);
            let new_guess = stop_offset + start_offset + self.distance;
            candidate.distance = candidate.distance.max(new_guess);
            if candidate.distance > best_distance {
                return false;
            }
        }
        true
    }
}

pub struct Cache {
    items: Vec<CacheItem>,
}

// Save start candidates and their valid end (stop) points. It is used to quickly determine (based on the stop sets and max distances of previous
// candidates) if a candidate can lead to a better result than the current best distances
impl Cache {
    pub fn new() -> Cache {
        Cache { items: Vec::new() }
    }
    pub fn set(&mut self, item: CacheItem) {
        self.items.push(item);
    }

    pub fn check(
        &mut self,
        candidate: &mut CacheItem,
        flat_points: &[FlatPoint<f32>],
        best_distance: f32,
    ) -> bool {
        // iterate in reverse order as it provides a speed-up on a broad test suite of files
        for cache_item in self.items.iter().rev() {
            if cache_item.places_upperbound(candidate, flat_points, best_distance) {
                return true;
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_item_with_super_set_does_not_place_upperbound() {
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let mut candidate = CacheItem {
            start: 0,
            stops: [0].into_iter().collect(),
            max_stop: 0,
            distance: 0.0,
        };

        // set the item with the super set in the cache
        let item = CacheItem {
            start: 0,
            max_stop: 0,
            stops: [0, 1].into_iter().collect(),
            distance: 0.0,
        };

        // set a high best distance to make sure the cache item stays below
        let best_distance = 1_000.0;
        assert!(!item.places_upperbound(&mut candidate, &flat_points, best_distance))
    }

    #[test]
    fn test_item_with_sub_set_places_upperbound() {
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let mut candidate = CacheItem {
            start: 0,
            stops: [0, 1].into_iter().collect(),
            max_stop: 1,
            distance: 0.0,
        };

        let item = CacheItem {
            start: 0,
            max_stop: 0,
            stops: [0].into_iter().collect(),
            distance: 0.0,
        };

        // set a high best distance to make sure the cache item stays below
        let best_distance = 1_000.0;
        assert!(item.places_upperbound(&mut candidate, &flat_points, best_distance));
    }

    #[test]
    fn test_item_with_sub_set_but_bigger_distance() {
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let mut candidate = CacheItem {
            start: 0,
            stops: [0, 1].into_iter().collect(),
            max_stop: 1,
            distance: 100.0,
        };

        let item = CacheItem {
            start: 0,
            max_stop: 0,
            stops: [0].into_iter().collect(),
            distance: 100.0,
        };

        // set a high best distance to make sure the item exceeds this
        let best_distance = 1.0;
        assert!(!item.places_upperbound(&mut candidate, &flat_points, best_distance))
    }

    #[test]
    fn test_set_preserves_order() {
        let mut cache = Cache::new();
        let first_item = CacheItem {
            start: 0,
            max_stop: 1,
            stops: HashSet::new(),
            distance: 0.0,
        };
        let second_item = CacheItem {
            start: 1,
            max_stop: 2,
            stops: HashSet::new(),
            distance: 0.0,
        };
        cache.set(first_item);
        cache.set(second_item);
        assert_eq!(cache.items.get(1).map(|item| item.start), Some(1));
    }

    #[test]
    fn test_empty_cache_returns_false() {
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let mut candidate = CacheItem {
            start: 0,
            stops: HashSet::new(),
            max_stop: 0,
            distance: 0.0,
        };
        let best_distance = 0.0;
        let mut cache = Cache::new();
        assert!(!cache.check(&mut candidate, &flat_points, best_distance));
    }

    #[test]
    fn test_cache_with_sub_set_item_returns_true() {
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let mut candidate = CacheItem {
            start: 0,
            stops: [0, 1].into_iter().collect(),
            max_stop: 1,
            distance: 0.0,
        };

        let item = CacheItem {
            start: 0,
            max_stop: 0,
            stops: [0].into_iter().collect(),
            distance: 0.0,
        };
        let mut cache = Cache::new();
        cache.set(item);

        // set a high best distance to make sure the cache item stays below
        let best_distance = 1_000.0;
        assert!(cache.check(&mut candidate, &flat_points, best_distance));
    }
}
