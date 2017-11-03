use sys;
use bullet_vector3::BulletVector3;
use collision::collision_shapes::Shape;
use mint::{Vector3, Vector4};

pub struct RigidBody {
    rigid_body: Box<sys::btRigidBody>,
    shape: Box<Shape>,
    motion_state: Box<sys::btDefaultMotionState>,
    construction_info: Box<sys::btRigidBody_btRigidBodyConstructionInfo>,
}

impl RigidBody {
    pub fn new<T1: Into<Vector3<f64>>, T2: Into<Vector3<f64>>, T3: Into<Vector4<f64>>>(
        mass: f64,
        inertia: T1,
        shape: Shape,
        translation: T2,
        orientation: T3,
    ) -> RigidBody {
        let mut inertia: BulletVector3 = inertia.into().into();
        let shape_box = Box::new(shape);
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
                shape_box.as_ptr(),
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
        }
    }

    pub(crate) unsafe fn motion_state_ptr(&self) -> *mut sys::btDefaultMotionState {
        &*self.motion_state as *const _ as *mut _
    }

    pub(crate) unsafe fn as_ptr(&self) -> *mut sys::btRigidBody {
        &*self.rigid_body as *const _ as *mut _
    }
}

#[derive(Clone)]
pub struct RigidBodyHandle {
    ptr: *mut sys::btRigidBody,
    motion_state: *mut sys::btDefaultMotionState,
    temp_transform: sys::btTransform,
}

impl RigidBodyHandle {
    pub fn new(
        ptr: *mut sys::btRigidBody,
        motion_state: *mut sys::btDefaultMotionState
    ) -> Self {
        let temp_transform = unsafe { sys::btTransform::new() };

        RigidBodyHandle {
            ptr,
            motion_state,
            temp_transform,
        }
    }
    pub fn set_restitution(&self, restitution: f64) {
        unsafe {
            sys::btCollisionObject_setRestitution(self.ptr as *mut _, restitution);
        }
    }

    pub fn set_gravity<T: Into<Vector3<f64>>>(&mut self, gravity: T) {
        let gravity: BulletVector3 = gravity.into().into();
        unsafe {
            sys::btRigidBody_setGravity(self.ptr, gravity.0.as_ptr() as *const _);
        }
    }

    pub fn set_angular_factor<T: Into<Vector3<f64>>>(&mut self, angular_factor: T) {
        let angular_factor: BulletVector3 = angular_factor.into().into();
        unsafe {
            sys::btRigidBody_setAngularFactor(self.ptr, angular_factor.0.as_ptr() as *const _);
        }
    }

    /// Get position in world space and orientation quaternion
    pub fn get_world_position_and_orientation(&self) -> (Vector3<f64>, Vector4<f64>) {
        unsafe {
            sys::btDefaultMotionState_getWorldTransform(
                &*self.motion_state as *const _ as *mut _,
                &self.temp_transform as *const _ as *mut _,
            );
        }
        let origin = unsafe { self.temp_transform.getOrigin1().as_ref().unwrap() };

        let rotation = unsafe { self.temp_transform.getRotation() };

        (
            Vector3::from_slice(&origin.m_floats[0..3]),
            Vector4::from_slice(&rotation._base.m_floats),
        )
    }

    /// Place data on heap as a box and set to rigid body as user pointer
    pub fn set_user_data<T: 'static>(&self, data: T) {
        let data_box = Box::new(data);
        unsafe {
            sys::btCollisionObject_setUserPointer(
                self.ptr as *mut _,
                Box::into_raw(data_box) as *mut _,
            )
        };
    }

    /// Get data from rigidbody's user pointer
    /// Getting data from body without previously setted data is fine.
    /// Getting wrong typed data is unsafe and will cause mem::transmute to wrong pointer type.
    pub unsafe fn get_user_data<T: 'static>(&self) -> Option<&T> {
        let pointer = sys::btCollisionObject_getUserPointer(self.ptr as *mut _);
        let pointer: *const T = ::std::mem::transmute(pointer);
        return pointer.as_ref();
    }
}
