extern crate cgmath;
extern crate bulletrs;

use cgmath::{Vector3, Vector4};
use bulletrs::*;

#[test()]
fn reset_linear_velocity() {
    let configuration = CollisionConfiguration::new_default();

    let mut dynamics_world = DynamicsWorld::new_discrete_world(
        CollisionDispatcher::new(&configuration),
        Broadphase::new(BroadphaseInterface::DbvtBroadphase),
        ConstraintSolver::new(),
        configuration,
    );

    let shape = Shape::new_sphere(1.0);
    let mut rb = dynamics_world.add_rigid_body(RigidBody::new(0.0,Vector3::new(0.0, 0.0, 0.0),
        shape,
        Vector3::new(0.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0)));
    rb.reset_linear_velocity(Vector3::new(1.0, 2.0, 3.0));
    let v: Vector3<f64> = rb.get_linear_velocity().into();
    assert_eq!(v, Vector3::new(1.0,2.0,3.0));
}