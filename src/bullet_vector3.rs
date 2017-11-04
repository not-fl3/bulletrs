use mint::{Vector3, Vector4};

pub struct BulletVector3(pub [f64; 4]);

impl Into<BulletVector3> for Vector3<f64> {
    fn into(self) -> BulletVector3 {
        BulletVector3([self.x, self.y, self.z, 0.0])
    }
}

pub fn vector_from_slice(slice : &[f64]) -> Vector3<f64> {
    Vector3 {
        x: slice[0],
        y: slice[1],
        z: slice[2]
    }
}

pub fn vector4_from_slice(slice : &[f64]) -> Vector4<f64> {
    Vector4 {
        x: slice[0],
        y: slice[1],
        z: slice[2],
        w: slice[3]
    }
}
