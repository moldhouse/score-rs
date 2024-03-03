use flat_projection::FlatPoint;
use std::collections::HashSet;

use crate::graph::StartCandidate;

pub struct Cache {
    cache: Vec<CacheItem>,
}

pub struct CacheItem {
    pub start: usize,
    pub last_stop: usize,
    pub stop_set: HashSet<usize>,
    pub distance: f32,
}

// Save start candidates and their valid end (stop) points. It is used to quickly determine (based on the stop sets and max distances of previous
// candidates) if a candidate can lead to a better result than the current best distances
impl Cache {
    pub fn new() -> Cache {
        Cache { cache: Vec::new() }
    }
    pub fn set(&mut self, item: CacheItem) {
        self.cache.push(item);
    }

    // If the current stop set is a super set of the stops of an item in the cache,
    // we can place an upper bound on the possible distance with the current stop set:
    //
    // The sum of:
    // 1. Offset start: The distance between the two start candidates
    // 2. The max distance of the cached item
    // 3. The max distance of any stop in the current stop set (that is not in the cached stop set)
    //    to the last (or any single other) item in the cached stop set (the last leads to the lowest bound in the most cases)
    pub fn check(
        &self,
        flat_points: &[FlatPoint<f32>],
        candidate: &StartCandidate,
        best_distance: f32,
        stop_set: &HashSet<usize>,
    ) -> bool {
        // iterate in reverse order as it proved to be more likely to find a match sooner
        for cache_item in self.cache.iter().rev() {
            let offset_start =
                flat_points[cache_item.start].distance(&flat_points[candidate.start_index]);
            if cache_item.distance + offset_start >= best_distance {
                // this item does not allow to find an upper bound below best_distance as 1. + 2. is already larger
                continue;
            }
            if !stop_set.is_superset(&cache_item.stop_set) {
                continue;
            }
            let mut disregard = true;
            for to_check in stop_set.difference(&cache_item.stop_set) {
                let offset_end =
                    flat_points[cache_item.last_stop].distance(&flat_points[*to_check]);
                let distance = offset_end + offset_start + cache_item.distance;
                if distance > best_distance {
                    disregard = false;
                    // this item does not place an upper bound
                    break;
                }
            }
            if disregard {
                return true;
            }
        }
        false
    }
}
