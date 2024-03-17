use flat_projection::{FlatPoint, FlatProjection};

use crate::parallel::*;
use crate::point::Point;

// Calculate the mean of two angles, output range is [-180, 180]
fn circ_mean(a: f32, b: f32) -> f32 {
    let a = a.to_radians();
    let b = b.to_radians();
    let x = (a.cos() + b.cos()) / 2.;
    let y = (a.sin() + b.sin()) / 2.;
    y.atan2(x).to_degrees()
}

/// Projects all geographic points onto a flat surface for faster geodesic calculation
///
pub fn to_flat_points<T: Point>(route: &[T]) -> Vec<FlatPoint<f32>> {
    let center = route.center().unwrap();
    let proj = FlatProjection::new(center.0, center.1);

    opt_par_iter(route)
        .map(|fix| proj.project(fix.longitude(), fix.latitude()))
        .collect()
}

struct BBox {
    lon_min: f32,
    lon_max: f32,
    lat_min: f32,
    lat_max: f32,
}

impl BBox {
    fn extend<T: Point>(&mut self, point: &T) {
        self.lon_min = self.lon_min.min(point.longitude());
        self.lon_max = self.lon_max.max(point.longitude());
        self.lat_min = self.lat_min.min(point.latitude());
        self.lat_max = self.lat_max.max(point.latitude());
    }

    fn center_lat(&self) -> f32 {
        (self.lat_min + self.lat_max) / 2.
    }
    fn center_lon(&self) -> f32 {
        circ_mean(self.lon_min, self.lon_max)
    }
}

impl<T: Point> From<&T> for BBox {
    fn from(value: &T) -> Self {
        BBox {
            lon_min: value.longitude(),
            lon_max: value.longitude(),
            lat_min: value.latitude(),
            lat_max: value.latitude(),
        }
    }
}

impl<T: Point> TryFrom<&[T]> for BBox {
    type Error = &'static str;
    fn try_from(value: &[T]) -> Result<Self, Self::Error> {
        match value.first() {
            None => Err("Empty array"),
            Some(first) => {
                let mut bbox = Self::from(first);
                value.iter().skip(1).for_each(|point| bbox.extend(point));
                Ok(bbox)
            }
        }
    }
}

trait Center {
    fn center(&self) -> Option<(f32, f32)>;
}

impl Center for BBox {
    fn center(&self) -> Option<(f32, f32)> {
        Some((self.center_lon(), self.center_lat()))
    }
}

impl<T: Point> Center for [T] {
    fn center(&self) -> Option<(f32, f32)> {
        BBox::try_from(self).ok()?.center()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    impl Point for (f64, f64) {
        fn longitude(&self) -> f32 {
            self.0 as f32
        }
        fn latitude(&self) -> f32 {
            self.1 as f32
        }
        fn altitude(&self) -> i16 {
            0
        }
    }

    #[test]
    fn test_circ_mean() {
        assert_approx_eq!(circ_mean(0., 0.), 0., 1e-5);
        assert_approx_eq!(circ_mean(0., 90.0), 45., 1e-5);
        assert_approx_eq!(circ_mean(0., 180.), -90., 1e-5);
        assert_approx_eq!(circ_mean(0., -170.), -85., 1e-5);
        assert_approx_eq!(circ_mean(0., 90.), 45., 1e-5);
        assert_approx_eq!(circ_mean(0., -90.), -45., 1e-5);
        assert_approx_eq!(circ_mean(-170., 178.), -176., 1e-5);
        assert_approx_eq!(circ_mean(-178., 170.), 176., 1e-5);
        assert_approx_eq!(circ_mean(90., -180.), 135., 1e-5);
    }
    #[test]
    fn test_bbox_from_point() {
        let point = (10., 50.0);
        let bbox = BBox::from(&point);
        assert_eq!(bbox.lon_min, 10.);
        assert_eq!(bbox.lon_max, 10.);
        assert_eq!(bbox.lat_min, 50.);
        assert_eq!(bbox.lat_max, 50.);
    }
    #[test]
    fn test_bbox_from_points() {
        let points: Vec<(f64, f64)> = vec![];
        assert!(points.center().is_none());

        let points = vec![(10., 50.0), (11., 51.), (12., 52.), (-10., -5.), (11., -5.)];

        let bbox = BBox::try_from(&points[..]).unwrap();
        assert_approx_eq!(bbox.lon_min, -10.);
        assert_approx_eq!(bbox.lon_max, 12.);
        assert_approx_eq!(bbox.lat_min, -5.);
        assert_approx_eq!(bbox.lat_max, 52.);
    }
    #[test]
    fn test_bbox_center() {
        let bbox = BBox {
            lon_min: -10.,
            lon_max: 12.,
            lat_min: -5.,
            lat_max: 52.,
        };

        let center = bbox.center().unwrap();
        assert_approx_eq!(center.0, 1.0);
        assert_approx_eq!(center.1, 23.5);
    }

    #[test]
    fn test_bbox_longitude_overflow() {
        let points = vec![(179., 50.), (-179., 50.)];
        let center = points.center().unwrap();
        assert_approx_eq!(center.0, 180.);
    }
}
