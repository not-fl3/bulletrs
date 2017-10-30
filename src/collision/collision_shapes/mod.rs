use sys;
use std::mem;

use mint::Vector3;

pub enum Shape {
    Sphere(sys::btSphereShape),
}

impl Shape {
    pub fn new_sphere(radius: f64) -> Shape {
        Shape::Sphere(unsafe { sys::btSphereShape::new(radius) })
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
            }
        }
    }
}
