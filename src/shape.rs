use mint::{Vector3, Vector4};

pub enum ShapeType {
    Sphere { radius: f64 },
    Cylinder,
    Box,
    Capsule,
    Plane { normal: Vector3<f64>, constant: f64 },
    TriMesh { vertices : Vec<Vector3<f64>>, scale : Vector3<f64> },
    Compound(Vec<(ShapeType, Vector3<f64>, Vector4<f64>)>),
}

#[derive(Clone, Copy)]
pub struct Shape {
    pub(crate) unique_id: i32,
}

impl ShapeType {
    pub(crate) fn create_shape(&self, command: ::sys::b3SharedMemoryCommandHandle) -> Option<i32> {
        match self {
            &ShapeType::Plane { normal, constant } => {
                let mut normal: [f64; 3] = normal.into();
                let shape_id = unsafe {
                    ::sys::b3CreateCollisionShapeAddPlane(
                        command,
                        &mut normal[0] as *mut f64,
                        constant,
                    )
                };
                Some(shape_id)
            }
            &ShapeType::Sphere { radius } => unsafe {
                let shape_id = ::sys::b3CreateCollisionShapeAddSphere(command, radius);
                Some(shape_id)
            },
            &ShapeType::TriMesh{ ref vertices, scale } => unsafe {
                let mut scale : [f64; 3] = scale.into();

                let shape_id = ::sys::b3CreateCollisionShapeAddTriMesh(
                    command,
                    vertices.len() as i32,
                    ::std::mem::transmute(vertices.as_ptr()),
                    (&mut scale).as_mut_ptr()
                );
                Some(shape_id)
            },

            &ShapeType::Compound(ref shapes) => {
                for &(ref shape, position, orientation) in shapes {
                    let shape_unique_id = shape
                        .create_shape(command)
                        .expect("Only one level of compound shapes supprted!");
                    let mut position: [f64; 3] = position.into();
                    let mut orientation: [f64; 4] = orientation.into();

                    unsafe {
                        ::sys::b3CreateCollisionShapeSetChildTransform(
                            command,
                            shape_unique_id,
                            &mut position[0] as *mut _,
                            &mut orientation[0] as *mut _,
                        );
                    }
                }
                None
            }
            _ => unimplemented!(),
        }
    }
}
