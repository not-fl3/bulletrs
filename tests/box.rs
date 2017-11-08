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

    dynamics_world.set_gravity(Vector3::new(0.0f64, -10.0f64, 1.0f64));

    let ground_shape = Shape::new_plane(Vector3::new(0.0, 1.0, 0.0), -2.0);

    let mut ground_rigid_body = dynamics_world.add_rigid_body(RigidBody::new(
        0.0,
        Vector3::new(0.0, 0.0, 0.0),
        ground_shape,
        Vector3::new(0.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));
    ground_rigid_body.set_restitution(0.95);

    let shape = Shape::new_box(Vector3::new(1.0,2.0,0.5));
    let mass = 0.1;
    let mut capsule = dynamics_world.add_rigid_body(RigidBody::new(
        mass,
        shape.calculate_local_inertia(mass),
        shape,
        Vector3::new(4.0, 2.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));
    capsule.set_angular_factor(Vector3::new(0.0, 0.0, 0.0));
    for _ in 0 .. 10 {
        dynamics_world.step_simulation(0.1, 0, 0.0);
    }

    let (_, orientation) = capsule.get_world_position_and_orientation();
    assert_eq!(orientation.x, 0.0);
    assert_eq!(orientation.y, 0.0);
    assert_eq!(orientation.z, 0.0);
    assert_eq!(orientation.w, 1.0);
}
