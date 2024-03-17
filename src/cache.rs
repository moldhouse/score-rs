use crate::graph::StartCandidate;
use flat_projection::FlatPoint;
use std::collections::HashSet;

pub struct Cache {
    items: Vec<CacheItem>,
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
        Cache { items: Vec::new() }
    }
    pub fn set(&mut self, item: CacheItem) {
        self.items.push(item);
    }

    // If the current stop set is a super set of the stops of an item in the cache,
    // we can place an upper bound on the possible distance with the current stop set:
    //
    // The sum of:
    // 1. Offset start: The distance between the two start candidates
    // 2. The max distance of the cached item
    // 3. The max distance of any stop in the current stop set (that is not in the cached stop set)
    //    to the last (or any single other) item in the cached stop set (the last leads to the lowest bound in the most cases)
    // If the upper bound is lower than the current best distance, we can rule out the candidate
    pub fn check(
        &mut self,
        flat_points: &[FlatPoint<f32>],
        candidate: &StartCandidate,
        best_distance: f32,
        stop_set: &HashSet<usize>,
    ) -> bool {
        // iterate in reverse order as it proved to be more likely to find a match sooner
        for cache_item in self.items.iter().rev() {
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
            let mut candidate_guess = cache_item.distance;
            for to_check in stop_set.difference(&cache_item.stop_set) {
                let offset_end =
                    flat_points[cache_item.last_stop].distance(&flat_points[*to_check]);
                let new_guess = offset_end + offset_start + cache_item.distance;
                if new_guess > candidate_guess {
                    candidate_guess = offset_end + offset_start + cache_item.distance;
                }
                if candidate_guess > best_distance {
                    disregard = false;
                    // this item does not place an upper bound
                    break;
                }
            }
            if disregard {
                // this information is usefull in the cache
                self.set(CacheItem {
                    start: candidate.start_index,
                    last_stop: stop_set.iter().max().unwrap().clone(),
                    stop_set: stop_set.clone(),
                    distance: candidate_guess,
                });
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
    fn test_set_preserves_order() {
        let mut cache = Cache::new();
        let first_item = CacheItem {
            start: 0,
            last_stop: 1,
            stop_set: HashSet::new(),
            distance: 0.0,
        };
        let second_item = CacheItem {
            start: 1,
            last_stop: 2,
            stop_set: HashSet::new(),
            distance: 0.0,
        };
        cache.set(first_item);
        cache.set(second_item);
        assert_eq!(cache.items.get(1).map(|item| item.start), Some(1));
    }

    #[test]
    fn test_empty_cache_returns_false() {
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let candidate = StartCandidate {
            start_index: 0,
            distance: 0.0,
        };
        let best_distance = 0.0;
        let stop_set = HashSet::new();

        let mut cache = Cache::new();
        assert_eq!(
            cache.check(&flat_points, &candidate, best_distance, &stop_set),
            false
        );
    }

    #[test]
    fn test_item_with_sub_set() {
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let candidate = StartCandidate {
            start_index: 0,
            distance: 0.0,
        };

        // set a high best distance to make sure the cache item stays below
        let best_distance = 1_000.0;

        let mut super_set: HashSet<_> = vec![0, 1].into_iter().collect();
        let sub_set: HashSet<_> = vec![0].into_iter().collect();
        super_set.insert(0);

        // set the item with the super set in the cache
        let mut cache = Cache::new();
        let item = CacheItem {
            start: 0,
            last_stop: 0,
            stop_set: super_set,
            distance: 0.0,
        };
        cache.set(item);

        assert_eq!(
            cache.check(&flat_points, &candidate, best_distance, &sub_set),
            false
        );
    }

    #[test]
    fn test_item_with_super_set() {
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let candidate = StartCandidate {
            start_index: 0,
            distance: 0.0,
        };

        // set a high best distance to make sure the cache item stays below
        let best_distance = 1_000.0;

        let mut super_set: HashSet<_> = vec![0, 1].into_iter().collect();
        let sub_set: HashSet<_> = vec![0].into_iter().collect();
        super_set.insert(0);

        // set the item with the super set in the cache
        let mut cache = Cache::new();
        let item = CacheItem {
            start: 0,
            last_stop: 0,
            stop_set: sub_set,
            distance: 0.0,
        };
        cache.set(item);

        assert_eq!(
            cache.check(&flat_points, &candidate, best_distance, &super_set),
            true
        );
    }

    #[test]
    fn test_item_with_super_set_but_bigger_distance() {
        pretty_env_logger::init();
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let candidate = StartCandidate {
            start_index: 0,
            distance: 100.0,
        };
        // set a high best distance to make sure the cache item stays
        let best_distance = 1.0;

        let mut super_set: HashSet<_> = vec![0, 1].into_iter().collect();
        let sub_set: HashSet<_> = vec![0].into_iter().collect();
        super_set.insert(0);

        // set the item with the super set in the cache
        let mut cache = Cache::new();
        let item = CacheItem {
            start: 0,
            last_stop: 0,
            stop_set: sub_set,
            distance: 100.0,
        };
        cache.set(item);

        assert_eq!(
            cache.check(&flat_points, &candidate, best_distance, &super_set),
            false
        );
    }
}
