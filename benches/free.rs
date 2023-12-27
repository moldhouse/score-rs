#[macro_use]
extern crate criterion;

extern crate igc;

use criterion::Criterion;
use igc::util::Time;
use score_rs::free;
use score_rs::point::PointImpl;

const LEGS: usize = 6;

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("free", |b| {
        b.iter(|| {
            let release = Time::from_hms(8, 12, 29);
            let fixes = include_str!("../tests/fixtures/2023-06-17_288167.igc")
                .lines()
                .filter(|l| l.starts_with('B'))
                .filter_map(|line| {
                    igc::records::BRecord::parse(&line)
                        .ok()
                        .map_or(None, |record| {
                            if record.timestamp.seconds_since_midnight()
                                >= release.seconds_since_midnight()
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
        })
    });
}

criterion_group! {
    name = benches;
    config = Criterion::default()
        .sample_size(10);

    targets = criterion_benchmark
}
criterion_main!(benches);
