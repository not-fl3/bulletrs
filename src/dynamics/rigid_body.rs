use sys;
use bullet_vector3::BulletVector3;
use collision::collision_shapes::Shape;
use mint::{Vector3, Vector4};

#[repr(u8)]
pub enum ActivationState {
    /// Means active so that the object having the state could be moved in a step simulation.
    /// This is the "normal" state for an object to be in.
    /// Use btCollisionObject::activate() to activate an object,
    /// not btCollisionObject::setActivationState(ACTIVATE_TAG),
    /// or it may get disabled again right away, as the deactivation timer has not been reset.
    ActiveTag = 1,
    /// Makes a body active forever, used for something like a player-controlled object.
    DisableDeactivation = 4,
    /// Making a body deactivated forever.
    DisableSimulation = 5,
    /// Means the body, and it's island, are asleep, since Bullet sleeps objects per-island. You probably don't want or need to set this one manually.
    IslandSleeping = 2,
    /// Means that it's an active object trying to fall asleep,
    /// and Bullet is keeping an eye on its velocity for the next few frames
    /// to see if it's a good candidate.
    /// You probably don't want or need to set this one manually.
    WantsDeactivation = 3,
}

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

impl Drop for RigidBody {
    fn drop(&mut self) {
        unsafe {
            ::sys::btMotionState_btMotionState_destructor(
                &mut *self.motion_state as *mut _ as *mut _,
            );
            ::sys::btRigidBody_btRigidBody_destructor(&mut *self.rigid_body as *mut _);
        }
    }
}

#[derive(Clone)]
pub struct RigidBodyHandle {
    pub(in dynamics) ptr: *mut sys::btRigidBody,
    motion_state: *mut sys::btDefaultMotionState,
    temp_transform: sys::btTransform,
}

impl RigidBodyHandle {
    pub fn new(ptr: *mut sys::btRigidBody, motion_state: *mut sys::btDefaultMotionState) -> Self {
        let temp_transform = unsafe { sys::btTransform::new() };

        RigidBodyHandle {
            ptr,
            motion_state,
            temp_transform,
        }
    }
    pub fn set_restitution(&mut self, restitution: f64) {
        unsafe {
            sys::btCollisionObject_setRestitution(self.ptr as *mut _, restitution);
        }
    }
    pub fn set_friction(&mut self, friction: f64) {
        unsafe {
            sys::btCollisionObject_setFriction(self.ptr as *mut _, friction);
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

    pub fn set_sleeping_thresholds(&mut self, linear: f64, angular: f64) {
        unsafe { sys::btRigidBody_setSleepingThresholds(self.ptr, linear, angular) }
    }

    pub fn set_activation_state(&mut self, activation_state: ActivationState) {
        unsafe {
            sys::btCollisionObject_setActivationState(self.ptr as *mut _, activation_state as i32)
        }
    }

    pub fn apply_central_impulse<T: Into<Vector3<f64>>>(&mut self, impulse: T) {
        let impulse: BulletVector3 = impulse.into().into();
        unsafe {
            sys::btRigidBody_applyCentralImpulse(self.ptr, impulse.0.as_ptr() as *const _);
        }
    }

    pub fn get_linear_velocity(&self) -> Vector3<f64> {
        let velocity = unsafe { sys::btRigidBody_getLinearVelocity(self.ptr) };

        ::bullet_vector3::vector_from_slice(unsafe {
            ::std::slice::from_raw_parts(velocity as *const _, 4)
        })
    }

    /// Override velocity vector.
    pub fn reset_linear_velocity<T>(&mut self, velocity: T)
    where
        T: Into<Vector3<f64>>,
    {
        let velocity: BulletVector3 = velocity.into().into();
        unsafe {
            sys::btRigidBody_setLinearVelocity(self.ptr, velocity.0.as_ptr() as *const _);
        }
    }

    /// Override position vector and rotation quaternion.
    pub fn reset_position_and_orientation<T, T1>(&mut self, position: T, orientation: T1)
    where
        T: Into<Vector3<f64>>,
        T1: Into<Vector4<f64>>,
    {
        let orientation: [f64; 4] = orientation.into().into();
        let position: BulletVector3 = position.into().into();
        let transform = unsafe {
            sys::btTransform::new1(
                &orientation as *const _ as *const _,
                &position as *const _ as *const _,
            )
        };

        unsafe {
            sys::btDefaultMotionState_setWorldTransform(
                &*self.motion_state as *const _ as *mut _,
                &transform as *const _ as *const _,
            );
        }

        unsafe { (*self.ptr)._base.m_worldTransform = transform };
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
            ::bullet_vector3::vector_from_slice(&origin.m_floats[0..3]),
            ::bullet_vector3::vector4_from_slice(&rotation._base.m_floats),
        )
    }

    /// Place data on heap as a box and set to rigid body as user pointer
    pub fn set_user_data<T: 'static>(&mut self, data: T) {
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

    /// Set user index. This will not be used in bullet and this is not related to user_data.
    pub fn set_user_index(&mut self, index: i32) {
        unsafe { sys::btCollisionObject_setUserIndex(self.ptr as *mut _, index) };
    }

    /// Get previously setted user index
    /// If index was not set - will return "-1"
    pub fn get_user_index(&self) -> i32 {
        unsafe { sys::btCollisionObject_getUserIndex(self.ptr as *mut _) }
    }

    /// Was that rigid_body removed with DynamicsWorld::remove_body()
    /// Probably may return true also for any not-added bodys
    pub fn removed(&self) -> bool {
        unsafe { (*self.ptr)._base.m_worldArrayIndex == -1 }
    }

    pub unsafe fn ptr(&mut self) -> *mut sys::btRigidBody {
        self.ptr
    }
}
