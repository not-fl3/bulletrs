use physics_client::PhysicsClientHandle;
use command::Command;
use errors::Error;

use std::any::Any;
use mint::{Point3, Vector3, Vector4};

pub struct RigidBody {
    pub deleted: bool,
    pub user_data: Option<Box<::std::any::Any>>,
    pub user_index: Option<i32>
}

impl RigidBody {
    pub fn new() -> Self {
        RigidBody {
            deleted: false,
            user_data: None,
            user_index: None
        }
    }
}
#[derive(Clone)]
pub struct RigidBodyHandle {
    pub(crate) rigid_body: *mut RigidBody,
    pub(crate) client_handle: PhysicsClientHandle,
    pub(crate) unique_id: i32,
}

impl RigidBodyHandle {
    /// Get the positions (x,y,z) and orientation (x,y,z,w) in quaternion
    /// values for the base link of your object
    /// Object is retrieved based on body index, which is the order
    /// the object was loaded into the simulation (0-based)
    pub fn get_base_position_and_orientation(&self) -> Result<(Vector3<f64>, Vector4<f64>), Error> {
        self.is_removed()?;

        self.client_handle.get_body_actual_state(self.clone()).map(
            |state| (state.position, state.orientation),
        )
    }

    pub fn get_linear_velocity(&self) -> Result<Vector3<f64>, Error> {
        self.is_removed()?;

        self.client_handle.get_body_actual_state(self.clone()).map(
            |state| (state.linear_velocity),
        )
    }

    pub fn set_angular_factor(&self, factor: Vector3<f64>) -> Result<(), Error> {
        self.is_removed()?;

        self.client_handle.submit_client_command_and_wait_status(
            &Command::SetAngularFactor(
                self.clone(),
                factor,
            ),
        );
        Ok(())
    }

    pub fn apply_central_impulse(&self, impulse: Vector3<f64>) -> Result<(), Error> {
        self.is_removed()?;

        self.client_handle.submit_client_command_and_wait_status(
            &Command::ApplyCentralImpulse(self.clone(), impulse),
        );
        Ok(())
    }

    /// Attach some user defined data to this rigid body.
    pub fn set_user_data(&self, data: Box<Any>) -> Result<(), Error> {
        self.is_removed()?;

        if let Some(rigid_body) = unsafe { self.rigid_body.as_mut() } {
           rigid_body.user_data = Some(data);
        }
        Ok(())
    }

    /// Get previously attached user defined number.
    pub fn get_user_data(&self) -> Result<&Box<Any>, Error> {
        self.is_removed()?;

        unsafe { self.rigid_body.as_ref() }.and_then(|rigid_body| rigid_body.user_data.as_ref().map(|user_data| &*user_data)).ok_or(Error::NoValue)
    }

    /// Attach some user defined number to this rigid body.
    /// User index and user data is completely unrelated and may be used separately.
    pub fn set_user_index(&self, data: i32) -> Result<(), Error> {
        self.is_removed()?;

        if let Some(rigid_body) = unsafe { self.rigid_body.as_mut() } {
           rigid_body.user_index = Some(data);
        }
        Ok(())
    }

    /// Get previously attached user defined number.
    pub fn get_user_index(&self) -> Result<i32, Error> {
        self.is_removed()?;

        unsafe { self.rigid_body.as_ref() }.and_then(|rigid_body| rigid_body.user_index).ok_or(Error::NoValue)
    }

    /// Set gravity to only this specific object
    /// Be carefull, this will affect only btRigidBody (not btMultiBody).
    pub fn set_body_gravity(&self, gravity: Vector3<f64>) -> Result<(), Error> {
        self.is_removed()?;

        self.client_handle.submit_client_command_and_wait_status(
            &Command::SetBodyGravity(
                self.clone(),
                gravity,
            ),
        );
        Ok(())
    }

    /// Set position and orientation of the object.
    /// Be carefull, it will affect collisions only after next simulation step
    /// And forces/velocities will be flushed.
    pub fn reset_position_and_orientation(&self, position: Point3<f64>, orientation: Vector4<f64>) -> Result<(), Error> {
        self.is_removed()?;

        self.client_handle.submit_client_command_and_wait_status(
            &Command::ResetBasePositionAndOrientation(self.clone(), position, orientation),
        );
        Ok(())
    }

    /// Was that body previously removed with "removed_rigid_body"
    pub fn removed(&self) -> bool {
        let rigid_body = unsafe { self.rigid_body.as_ref().unwrap() };
        rigid_body.deleted
    }

    fn is_removed(&self) -> Result<(), Error> {
        if self.removed() {
            return Err(Error::BodyDeleted);
        }
        return Ok(());
    }
}
