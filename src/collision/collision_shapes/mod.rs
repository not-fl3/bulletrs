use sys;
use std::mem;
use bullet_vector3::BulletVector3;
use mint::{Vector3, Vector4};

pub enum Shape {
    Sphere(sys::btSphereShape),
    Plane(sys::btStaticPlaneShape),
    Capsule(sys::btCapsuleShape),
    ConvexHull(sys::btConvexHullShape),
    Compound {
        shape: sys::btCompoundShape,
        child_shapes: Vec<Box<Shape>>,
    },
}

impl Shape {
    pub fn new_sphere(radius: f64) -> Shape {
        Shape::Sphere(unsafe { sys::btSphereShape::new(radius) })
    }

    pub fn new_plane<T: Into<Vector3<f64>>>(normal: T, plane_const: f64) -> Shape {
        let up: BulletVector3 = normal.into().into();
        Shape::Plane(unsafe {
            sys::btStaticPlaneShape::new(up.0.as_ptr() as *const _, plane_const)
        })
    }

    pub fn new_capsule(radius: f64, height : f64) -> Shape {
        Shape::Capsule(unsafe { sys::btCapsuleShape::new1(radius, height) })
    }

    pub fn new_convex_hull<T : Into<Vector3<f64>> + Clone>(vertices: &[T]) -> Shape {
        let mut shape = unsafe { sys::btConvexHullShape::new(::std::ptr::null(), 0, 8 * 4) };
        for vertex in vertices.iter() {
            let vertex : BulletVector3 = vertex.clone().into().into();
            unsafe { shape.addPoint(&vertex as *const _ as *const _, true); }
        }
        Shape::ConvexHull(shape)
    }

    pub fn new_compound<T, T1>(shapes: Vec<(Shape, T, T1)>) -> Shape
    where
        T: Into<Vector3<f64>>,
        T1: Into<Vector4<f64>>,
    {
        let mut child_shapes = vec![];
        let mut compound_shape = unsafe { sys::btCompoundShape::new(true, 0) };
        for (shape, position, orientation) in shapes.into_iter() {
            let shape_box = Box::new(shape);
            let orientation: [f64; 4] = orientation.into().into();
            let position: BulletVector3 = position.into().into();
            let transform = unsafe {
                sys::btTransform::new1(
                    &orientation as *const _ as *const _,
                    &position as *const _ as *const _,
                )
            };
            unsafe {
                compound_shape
                    .addChildShape(&transform as *const _, shape_box.as_ptr());
            }
            child_shapes.push(shape_box);
        }
        Shape::Compound {
            shape: compound_shape,
            child_shapes,
        }
    }

    pub(crate) fn as_ptr(&self) -> *mut sys::btCollisionShape {
        match self {
            &Shape::Sphere(ref shape) => shape as *const _ as *mut _,
            &Shape::Plane(ref shape) => shape as *const _ as *mut _,
            &Shape::Capsule(ref shape) => shape as *const _ as *mut _,
            &Shape::ConvexHull(ref shape) => shape as *const _ as *mut _,
            &Shape::Compound { ref shape, .. } => shape as *const _ as *mut _,
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
            }
            &Shape::Plane(ref sphere) => {
                unsafe {
                    sys::btStaticPlaneShape_calculateLocalInertia(
                        sphere as *const _ as *mut _,
                        mass,
                        inertia.as_mut_ptr() as *mut _,
                    );
                }
            }
            &Shape::Capsule(ref shape) => {
                unsafe {
                    sys::btCapsuleShape_calculateLocalInertia(
                        shape as *const _ as *mut _,
                        mass,
                        inertia.as_mut_ptr() as *mut _,
                    );
                }
            }

            &Shape::ConvexHull(ref shape) => {
                unsafe {
                    sys::btPolyhedralConvexShape_calculateLocalInertia(
                        shape as *const _ as *mut _,
                        mass,
                        inertia.as_mut_ptr() as *mut _,
                    );
                }
            }
            &Shape::Compound { ref shape, .. } => {
                unsafe {
                    sys::btCompoundShape_calculateLocalInertia(
                        shape as *const _ as *mut _,
                        mass,
                        inertia.as_mut_ptr() as *mut _,
                    );
                }
            }
        }
        ::bullet_vector3::vector_from_slice(&inertia[0..3])
    }
}
