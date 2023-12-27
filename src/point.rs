pub trait Point: Sync {
    fn latitude(&self) -> f32;
    fn longitude(&self) -> f32;
    fn altitude(&self) -> i16;
}

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
