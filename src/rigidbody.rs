use physics_client::PhysicsClientHandle;
use command::Command;
use errors::Error;

use mint::{Vector3, Vector4};

#[derive(Clone)]
pub struct RigidBodyHandle {
    pub(crate) client_handle: PhysicsClientHandle,
    pub(crate) unique_id: i32,
}

impl RigidBodyHandle {
    /// Get the positions (x,y,z) and orientation (x,y,z,w) in quaternion
    /// values for the base link of your object
    /// Object is retrieved based on body index, which is the order
    /// the object was loaded into the simulation (0-based)
    pub fn get_base_position_and_orientation(&self) -> Result<(Vector3<f64>, Vector4<f64>), Error> {
        self.client_handle.get_body_actual_state(self.clone()).map(|state| (state.position, state.orientation))
    }

    pub fn get_linear_velocity(&self) -> Result<Vector3<f64>, Error> {
        self.client_handle.get_body_actual_state(self.clone()).map(|state| (state.linear_velocity))
    }

    pub fn set_angular_factor(&self, factor : Vector3<f64>) {
        self.client_handle.submit_client_command_and_wait_status(
            &Command::SetAngularFactor(self.clone(), factor),
        );
    }

    pub fn apply_central_impulse(&self, impulse : Vector3<f64>) {
        self.client_handle.submit_client_command_and_wait_status(
            &Command::ApplyCentralImpulse(self.clone(), impulse),
        );
    }

    pub fn set_user_data<T : 'static>(&self, data: Box<T>) {
        self.client_handle.set_user_data(self.clone(), data);
    }

    pub fn get_user_data<T : 'static>(&self) -> Result<&T, Error> {
        self.client_handle.get_user_data(self.clone())
    }

    /// Set gravity to only this specific object
    /// Be carefull, this will affect only btRigidBody (not btMultiBody).
    pub fn set_body_gravity(&self, gravity : Vector3<f64>) {
        self.client_handle.submit_client_command_and_wait_status(
            &Command::SetBodyGravity(self.clone(), gravity),
        );
    }

}
