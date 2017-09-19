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
    pub(crate) unique_id: i32,
}
