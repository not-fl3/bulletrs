use mint::Vector3;

pub struct BulletVector3(pub [f64; 4]);

impl Into<BulletVector3> for Vector3<f64> {
    fn into(self) -> BulletVector3 {
        BulletVector3([self.x, self.y, self.z, 0.0])
    }
}

