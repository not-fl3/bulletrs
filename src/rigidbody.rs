use physics_client::PhysicsClientHandle;
use command::Command;

use mint::{Vector3};

#[derive(Clone)]
pub struct RigidBodyHandle {
    pub(crate) client_handle: PhysicsClientHandle,
    pub(crate) unique_id: i32,
}

impl RigidBodyHandle {
    pub fn set_angular_factor(&self, factor : Vector3<f64>) {
        self.client_handle.submit_client_command_and_wait_status(
            &Command::SetAngularFactor(self.clone(), factor),
        );
    }
}
