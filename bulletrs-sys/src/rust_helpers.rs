use bt_bullet_dynamics_common::*;

extern "C" {
    #[link_name = "\u{1}_Z21newDefaultMotionStateRK11btTransform"]
    pub fn newDefaultMotionState(trans: *const btTransform) -> btDefaultMotionState;
}

extern "C" {
    #[link_name = "\u{1}_Z14newSphereShaped"]
    pub fn newSphereShape(radius: btScalar) -> btSphereShape;
}
