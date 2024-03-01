use flat_projection::{FlatPoint, FlatProjection};
use ord_subset::OrdSubsetIterExt;

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
pub fn to_flat_points<T: Point>(points: &[T]) -> Vec<FlatPoint<f32>> {
    let center_lat = points.center_lat().unwrap();
    let center_lon = points.center_lon().unwrap();
    let proj = FlatProjection::new(center_lon, center_lat);

    opt_par_iter(points)
        .map(|fix| proj.project(fix.longitude(), fix.latitude()))
        .collect()
}

trait CenterLatitude {
    fn center_lat(&self) -> Option<f32>;
}

impl<T: Point> CenterLatitude for [T] {
    fn center_lat(&self) -> Option<f32> {
        let lat_min = self.iter().map(|fix| fix.latitude()).ord_subset_min()?;
        let lat_max = self.iter().map(|fix| fix.latitude()).ord_subset_max()?;

        Some((lat_min + lat_max) / 2.)
    }
}

trait CenterLongitude {
    fn center_lon(&self) -> Option<f32>;
}

impl<T: Point> CenterLongitude for [T] {
    fn center_lon(&self) -> Option<f32> {
        let lon_min = self.iter().map(|fix| fix.longitude()).ord_subset_min()?;
        let lon_max = self.iter().map(|fix| fix.longitude()).ord_subset_max()?;

        Some(circ_mean(lon_min, lon_max))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    impl Point for (f32, f32) {
        fn latitude(&self) -> f32 {
            self.0
        }
        fn longitude(&self) -> f32 {
            self.1
        }
        fn altitude(&self) -> i16 {
            0
        }
    }

    #[test]
    fn test_circ_mean() {
        assert_eq!(circ_mean(0., 0.), 0.);
        assert_eq!(circ_mean(0., 90.0), 45.);
        assert_eq!(circ_mean(0., 180.), -90.);
        assert_eq!(circ_mean(0., -170.), -85.);
        assert_eq!(circ_mean(0., 90.), 45.);
        assert_eq!(circ_mean(0., -90.), -45.);
        assert_eq!(circ_mean(-170., 178.), -176.);
        assert_eq!(circ_mean(-178., 170.), 176.);
        assert_eq!(circ_mean(90., -180.), 135.);
    }
    #[test]
    fn test_center() {
        let points = vec![(50., 10.), (51., 11.), (52., 12.), (-5., -10.), (-5., 11.)];

        assert_approx_eq!(points.center_lat().unwrap(), 23.5);
        assert_approx_eq!(points.center_lon().unwrap(), 1.0);
    }

    #[test]
    fn test_longitude_overflow() {
        let points = vec![(50., 179.), (50., -179.)];
        assert_approx_eq!(points.center_lon().unwrap(), 180.);
    }
}
