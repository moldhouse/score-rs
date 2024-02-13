use numpy::PyReadonlyArray1;
use pyo3::prelude::*;

pub mod flat;
pub mod free;
pub mod parallel;
pub mod point;
pub mod utils;
pub mod vincenty;

#[pymodule]
fn score_rs(_py: Python, m: &PyModule) -> PyResult<()> {
    #[pyfn(m)]
    #[pyo3(name = "optimize")]
    fn optimize_py<'py>(
        longitude: PyReadonlyArray1<'py, f64>,
        latitude: PyReadonlyArray1<'py, f64>,
        alt: PyReadonlyArray1<'py, i64>,
        legs: usize,
    ) -> PyResult<(Vec<usize>, f32)> {
        let mut points = Vec::new();
        let longitude = longitude.as_slice().unwrap();
        let latitude = latitude.as_slice().unwrap();
        let alt = alt.as_slice().unwrap();
        for i in 0..longitude.len() {
            points.push(point::PointImpl {
                longitude: longitude[i] as f32,
                latitude: latitude[i] as f32,
                altitude: alt[i] as i16,
            });
        }
        let result = free::optimize(&points, 0.0, legs).unwrap();
        Ok((result.path, result.distance))
    }
    Ok(())
}
