extern crate bulletrs;
extern crate cgmath;

use cgmath::{Vector3, Vector4};

use bulletrs::*;

#[test()]
fn set_gravity() {
    let configuration = CollisionConfiguration::new_default();

    let mut dynamics_world = DynamicsWorld::new_discrete_world(
        CollisionDispatcher::new(&configuration),
        Broadphase::new(BroadphaseInterface::DbvtBroadphase),
        ConstraintSolver::new(),
        configuration,
    );

    dynamics_world.set_gravity(Vector3::new(0.0, -10.0, 0.0));

    let shape = Shape::new_sphere(2.0);
    let mass = 0.1;
    let body1 = dynamics_world.add_rigid_body(RigidBody::new(
        mass,
        shape.calculate_local_inertia(mass),
        shape,
        Vector3::new(4.0, 2.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));

    let shape2 = Shape::new_sphere(2.0);
    let mass = 0.1;
    let mut body2 = dynamics_world.add_rigid_body(RigidBody::new(
        mass,
        shape2.calculate_local_inertia(mass),
        shape2,
        Vector3::new(0.0, 2.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));
    body2.set_gravity(Vector3::new(0.0, 0.0, 0.0));

    for _ in 0 .. 10 {
        dynamics_world.step(0.1, 0, 0.0);
    }

    let (position, _) = body1.get_world_position_and_orientation();
    assert!(position.y != 2.0);

    let (position, _) = body2.get_world_position_and_orientation();
    assert_eq!(position.y, 2.0);
}
