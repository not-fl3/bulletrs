use sys;
use bullet_vector3::BulletVector3;
use collision::collision_shapes::Shape;
use mint::{Vector3, Vector4};

pub struct RigidBody {
    rigid_body: Box<sys::btRigidBody>,
    shape: Box<Shape>,
    motion_state: Box<sys::btDefaultMotionState>,
    construction_info: Box<sys::btRigidBody_btRigidBodyConstructionInfo>,
    temp_transform: sys::btTransform,
}

impl RigidBody {
    pub fn new<T1: Into<Vector3<f64>>, T2: Into<Vector3<f64>>, T3: Into<Vector4<f64>>>(
        mass: f64,
        shape: Shape,
        inertia: T1,
        translation: T2,
        orientation: T3,
    ) -> RigidBody {
        let mut inertia: BulletVector3 = inertia.into().into();
        let mut shape_box = Box::new(shape);
        let translation: BulletVector3 = translation.into().into();
        let orientation: [f64; 4] = orientation.into().into();
        let transform = unsafe {
            sys::btTransform::new1(
                &orientation as *const _ as *const _,
                &translation as *const _ as *const _,
            )
        };
        let mut motion_state_box = Box::new(unsafe {
            sys::btDefaultMotionState::new(&transform as *const _, sys::btTransform::getIdentity())
        });

        let construction_info_box = Box::new(unsafe {
            sys::btRigidBody_btRigidBodyConstructionInfo::new(
                mass,
                &mut *motion_state_box as *mut _ as *mut _,
                &mut *shape_box as *mut _ as *mut _,
                inertia.0.as_mut_ptr() as *mut _,
            )
        });
        RigidBody {
            rigid_body: Box::new(unsafe {
                sys::btRigidBody::new(&*construction_info_box as *const _)
            }),
            shape: shape_box,
            motion_state: motion_state_box,
            construction_info: construction_info_box,
            temp_transform: unsafe { sys::btTransform::new() },
        }
    }

    pub fn as_ptr(&self) -> *mut sys::btRigidBody {
        &*self.rigid_body as *const _ as *mut _
    }


    pub fn get_world_transform(&self) -> Vector3<f64> {
        unsafe {
            sys::btDefaultMotionState_getWorldTransform(
                &*self.motion_state as *const _ as *mut _,
                &self.temp_transform as *const _ as *mut _,
            );
        }
        let origin = unsafe { self.temp_transform.getOrigin1().as_ref().unwrap() };

        Vector3::from_slice(&origin.m_floats[0..3])
    }
}
