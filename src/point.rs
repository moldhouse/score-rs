use crate::vincenty::vincenty_distance;
use flat_projection::FlatPoint;
pub trait Point: Sync {
    fn latitude(&self) -> f32;
    fn longitude(&self) -> f32;
    fn altitude(&self) -> i16;
}
#[derive(Clone)]
pub struct PointImpl {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: i16,
}

impl Point for PointImpl {
    fn latitude(&self) -> f32 {
        self.latitude
    }
    fn longitude(&self) -> f32 {
        self.longitude
    }
    fn altitude(&self) -> i16 {
        self.altitude
    }
}

pub trait Valid {
    fn valid(&self, start: usize, stop: usize) -> bool;
}

impl<T: Point> Valid for [T] {
    fn valid(&self, start: usize, stop: usize) -> bool {
        self[start].altitude() - self[stop].altitude() <= 1000
    }
}

pub trait ApproxDistance {
    fn distance(&self, start: usize, stop: usize) -> f32;
    fn cum_distance(&self, path: &Path) -> f32;
}

impl ApproxDistance for [FlatPoint<f32>] {
    fn distance(&self, start: usize, stop: usize) -> f32 {
        self[start].distance(&self[stop])
    }
    fn cum_distance(&self, path: &Path) -> f32 {
        path.iter()
            .zip(path.iter().skip(1))
            .map(|(i1, i2)| self.distance(*i1, *i2))
            .sum()
    }
}

pub type Path = Vec<usize>;

pub trait VincentyDistance {
    fn cum_distance(&self, path: &Path) -> f32;
}

impl<T: Point> VincentyDistance for &[T] {
    fn cum_distance(&self, path: &Path) -> f32 {
        path.iter()
            .zip(path.iter().skip(1))
            .map(|(i1, i2)| (&self[*i1], &self[*i2]))
            .map(|(fix1, fix2)| vincenty_distance(fix1, fix2))
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;

    use super::*;
    #[test]
    fn route_valid_for_negative_alt_diff() {
        let points = vec![
            PointImpl {
                latitude: 0.0,
                longitude: 0.0,
                altitude: -1000,
            },
            PointImpl {
                latitude: 0.0,
                longitude: 0.0,
                altitude: 0,
            },
        ];
        assert!(points.valid(0, 1));
    }

    #[test]
    fn route_not_valid_for_high_alt_diff() {
        let points = vec![
            PointImpl {
                latitude: 0.0,
                longitude: 0.0,
                altitude: 0,
            },
            PointImpl {
                latitude: 0.0,
                longitude: 0.0,
                altitude: 2_000,
            },
        ];
        assert!(points.valid(0, 1));
    }

    #[test]
    fn approx_distance_between_two_points() {
        let points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        assert_approx_eq!(points.distance(0, 1), 1.41, 0.01);
    }

    #[test]
    fn approx_distance_same_points_is_zero() {
        let points = vec![FlatPoint { x: 0.0, y: 0.0 }, FlatPoint { x: 1.0, y: 1.0 }];
        assert_eq!(points.distance(0, 0), 0.0);
    }

    #[test]
    fn approx_cum_distance_adds_up() {
        let points = vec![
            FlatPoint { x: 0.0, y: 0.0 },
            FlatPoint { x: 0.0, y: 1.0 },
            FlatPoint { x: 3.0, y: 5.0 },
        ];
        let path = vec![0, 1, 2];
        assert_approx_eq!(points.cum_distance(&path), 6.00, 0.01);
    }

    #[test]
    fn cumulative_vincenty_distance_adds_up() {
        let points = vec![
            PointImpl {
                latitude: 50.0,
                longitude: 10.0,
                altitude: 0,
            },
            PointImpl {
                latitude: 51.0,
                longitude: 11.0,
                altitude: 0,
            },
            PointImpl {
                latitude: 52.0,
                longitude: 12.0,
                altitude: 0,
            },
        ];
        let path = vec![0, 1, 2];
        assert_approx_eq!(&points.as_slice().cum_distance(&path), 263.08, 0.01);
    }
}
