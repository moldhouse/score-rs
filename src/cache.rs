use crate::graph::StartCandidate;
use flat_projection::FlatPoint;
use std::collections::HashSet;

pub struct CacheItem {
    pub start: usize,
    pub last_stop: usize,
    pub stop_set: HashSet<usize>,
    pub distance: f32,
}

impl CacheItem {
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
        flat_points: &[FlatPoint<f32>],
        candidate: &StartCandidate,
        best_distance: f32,
        stop_set: &HashSet<usize>,
    ) -> Option<f32> {
        let offset_start = flat_points[self.start].distance(&flat_points[candidate.start_index]);
        let mut candidate_guess = self.distance + offset_start;
        if candidate_guess >= best_distance {
            // this item does not provide an upper bound below best_distance
            return None;
        }
        if !stop_set.is_superset(&self.stop_set) {
            return None;
        }
        for to_check in stop_set.difference(&self.stop_set) {
            let offset_end = flat_points[self.last_stop].distance(&flat_points[*to_check]);
            let new_guess = offset_end + offset_start + self.distance;
            candidate_guess = candidate_guess.max(new_guess);
            if candidate_guess > best_distance {
                return None;
            }
        }
        Some(candidate_guess)
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
        flat_points: &[FlatPoint<f32>],
        candidate: &StartCandidate,
        best_distance: f32,
        stop_set: &HashSet<usize>,
    ) -> bool {
        // iterate in reverse order as it provides a speed-up on a broad test suite of files
        for cache_item in self.items.iter().rev() {
            if let Some(upperbound) =
                cache_item.places_upperbound(flat_points, candidate, best_distance, stop_set)
            {
                // there is no need to add this to the cache, because the relation is transitive
                // if A provides an upperbound for B, and B provides an upperbound for a later C
                // then A provides an upperbound for C, so we don't need to add B to the cache
                //
                // BUT: adding this to the cache provides a speed-up on the test suite
                self.set(CacheItem {
                    start: candidate.start_index,
                    last_stop: *stop_set.iter().max().unwrap(),
                    stop_set: stop_set.clone(),
                    distance: upperbound,
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
    fn test_item_with_super_set_does_not_place_upperbound() {
        // set a high best distance to make sure the cache item stays below
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let candidate = StartCandidate {
            start_index: 0,
            distance: 0.0,
        };
        let best_distance = 1_000.0;

        let mut super_set: HashSet<_> = vec![0, 1].into_iter().collect();
        let sub_set: HashSet<_> = vec![0].into_iter().collect();
        super_set.insert(0);

        // set the item with the super set in the cache
        let item = CacheItem {
            start: 0,
            last_stop: 0,
            stop_set: super_set,
            distance: 0.0,
        };

        assert_eq!(
            item.places_upperbound(&flat_points, &candidate, best_distance, &sub_set),
            None
        );
    }

    #[test]
    fn test_item_with_sub_set_places_upperbound() {
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

        let item = CacheItem {
            start: 0,
            last_stop: 0,
            stop_set: sub_set,
            distance: 0.0,
        };
        assert!(item
            .places_upperbound(&flat_points, &candidate, best_distance, &super_set)
            .is_some());
    }

    #[test]
    fn test_item_with_sub_set_but_bigger_distance() {
        let flat_points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        let candidate = StartCandidate {
            start_index: 0,
            distance: 100.0,
        };
        // set a high best distance to make sure the item exceeds this
        let best_distance = 1.0;

        let mut super_set: HashSet<_> = vec![0, 1].into_iter().collect();
        let sub_set: HashSet<_> = vec![0].into_iter().collect();
        super_set.insert(0);

        let item = CacheItem {
            start: 0,
            last_stop: 0,
            stop_set: sub_set,
            distance: 100.0,
        };

        assert!(item
            .places_upperbound(&flat_points, &candidate, best_distance, &super_set)
            .is_none());
    }

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
    fn test_cache_with_sub_set_item_returns_true() {
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
}
