use sys;
use std::mem;
use bullet_vector3::BulletVector3;
use mint::Vector3;

pub enum Shape {
    Sphere(sys::btSphereShape),
    Plane(sys::btStaticPlaneShape),
}

impl Shape {
    pub fn new_sphere(radius: f64) -> Shape {
        Shape::Sphere(unsafe { sys::btSphereShape::new(radius) })
    }

    pub fn new_plane<T : Into<Vector3<f64>>>(normal : T, plane_const: f64) -> Shape {
        let up: BulletVector3 = normal.into().into();
        Shape::Plane(unsafe { sys::btStaticPlaneShape::new(up.0.as_ptr() as *const _, plane_const) })
    }

    pub fn as_ptr(&self) -> *mut sys::btCollisionShape {
        match self {
            &Shape::Sphere(ref sphere) => sphere as *const _ as *mut _,
            &Shape::Plane(ref plane) => plane as *const _ as *mut _,
        }
    }

    pub fn calculate_local_inertia(&self, mass: f64) -> Vector3<f64> {
        let mut inertia: [f64; 4] = unsafe { mem::uninitialized() };
        match self {
            &Shape::Sphere(ref sphere) => {
                unsafe {
                    sys::btSphereShape_calculateLocalInertia(
                        sphere as *const _ as *mut _,
                        mass,
                        inertia.as_mut_ptr() as *mut _,
                    );
                }
                Vector3::from_slice(&inertia[0..3])
            },
            &Shape::Plane(ref sphere) => {
                unsafe {
                    sys::btStaticPlaneShape_calculateLocalInertia(
                        sphere as *const _ as *mut _,
                        mass,
                        inertia.as_mut_ptr() as *mut _,
                    );
                }
                Vector3::from_slice(&inertia[0..3])
            }
        }
    }
}
