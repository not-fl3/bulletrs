use physics_client::PhysicsClientHandle;
use errors::Error;

use mint::{Vector3, Vector4};

#[derive(Clone, Default)]
pub struct DynamicsInfo {
    /// change the mass of the link (or base for linkIndex -1)
    pub mass: Option<f64>,

    /// lateral (linear) contact friction
    pub lateral_friction: Option<f64>,

    /// torsional friction around the contact normal
    pub spinning_friction: Option<f64>,

    /// torsional friction orthogonal to contact normal
    pub rolling_friction: Option<f64>,

    /// bouncyness of contact. Keep it a bit less than 1.
    pub restitution: Option<f64>,

    pub linear_damping: Option<f64>,
    pub angular_damping: Option<f64>,

    pub contact_stiffness_and_damping: Option<(f64, f64)>,
    pub friction_anchor: Option<i32>,
}

#[derive(Clone)]
pub struct MultiBodyHandle {
    pub(crate) client_handle: PhysicsClientHandle,
    pub(crate) unique_id: i32,
}

impl MultiBodyHandle {
    /// Get the positions (x,y,z) and orientation (x,y,z,w) in quaternion
    /// values for the base link of your object
    /// Object is retrieved based on body index, which is the order
    /// the object was loaded into the simulation (0-based)
    pub fn get_base_position_and_orientation(&self) -> Result<(Vector3<f64>, Vector4<f64>), Error> {
        self.client_handle.get_base_position_and_orientation(self.clone())
    }
}
