#[macro_use]
extern crate assert_approx_eq;
extern crate igc;

use igc::util::Time;
use score_rs::free;
use score_rs::free::OptimizationResult;
use score_rs::point::PointImpl;

const LEGS: usize = 6;

#[test]
fn free_distance() {
    let release = Time::from_hms(8, 12, 29);
    let result = run_free_test(include_str!("fixtures/2023-06-17_288167.igc"), release);
    assert_approx_eq!(result.distance, 1018.5, 0.1);
    assert_eq!(result.path, [0, 936, 2847, 3879, 5048, 7050, 8128]);
}

#[test]
fn free_distance_with_1000m() {
    let release = Time::from_hms(8, 16, 30);
    let result = run_free_test(include_str!("fixtures/schunk_1000m.igc"), release);
    assert_approx_eq!(result.distance, 1158.61, 0.1);
    assert_eq!(result.path, [335, 10099, 14740, 15482, 24198, 34160, 35798]);
}

fn run_free_test(file: &str, release: Time) -> OptimizationResult {
    env_logger::try_init().ok();

    let fixes = file
        .lines()
        .filter(|l| l.starts_with('B'))
        .filter_map(|line| {
            igc::records::BRecord::parse(&line)
                .ok()
                .map_or(None, |record| {
                    if record.timestamp.seconds_since_midnight() >= release.seconds_since_midnight()
                    {
                        Some(PointImpl {
                            latitude: record.pos.lat.into(),
                            longitude: record.pos.lon.into(),
                            altitude: record.pressure_alt,
                        })
                    } else {
                        None
                    }
                })
        })
        .collect::<Vec<_>>();

    free::optimize(&fixes, 0.0, LEGS).unwrap()
}
