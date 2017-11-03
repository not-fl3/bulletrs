extern crate bulletrs;
extern crate cgmath;

use cgmath::{Vector3, Vector4};

use bulletrs::*;

fn main() {
    let configuration = CollisionConfiguration::new_default();

    let mut dynamics_world = DynamicsWorld::new_discrete_world(
        CollisionDispatcher::new(&configuration),
        Broadphase::new(BroadphaseInterface::DbvtBroadphase),
        ConstraintSolver::new(),
        configuration,
    );

    dynamics_world.set_gravity(Vector3::new(0.0, -10.0, 0.0));

    let ground_shape = Shape::new_plane(Vector3::new(0.0, 1.0, 0.0), -2.0);
    let ground_rigid_body = dynamics_world.add_rigid_body(RigidBody::new(
        0.0,
        Vector3::new(0.0, 0.0, 0.0),
        ground_shape,
        Vector3::new(0.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));
    ground_rigid_body.set_restitution(0.95);

    let fall_shape = Shape::new_sphere(2.0);
    let mass = 0.1;
    let fall_rigid_body = dynamics_world.add_rigid_body(RigidBody::new(
        mass,
        fall_shape.calculate_local_inertia(mass),
        fall_shape,
        Vector3::new(0.0, 5.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));
    fall_rigid_body.set_restitution(0.9);

    for _ in 0..100 {
        dynamics_world.step_simulation(0.1, 0, 0.0);
        println!("{:?}", fall_rigid_body.get_world_position_and_orientation());
    }
}
