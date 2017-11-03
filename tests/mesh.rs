extern crate bulletrs;
extern crate cgmath;

use cgmath::{Vector3, Vector4};

use bulletrs::*;

#[test()]
fn compound_mesh() {
    let configuration = CollisionConfiguration::new_default();

    let mut dynamics_world = DynamicsWorld::new_discrete_world(
        CollisionDispatcher::new(&configuration),
        Broadphase::new(BroadphaseInterface::DbvtBroadphase),
        ConstraintSolver::new(),
        configuration,
    );

    dynamics_world.set_gravity(Vector3::new(0.0, -10.0, 0.0));

    let shape = Shape::new_compound(vec![
        (
            Shape::new_sphere(2.0),
            Vector3::new(-5.0, 0.0, 0.0),
            Vector4::new(0.0, 0.0, 0.0, 1.0),
        ),
        (
            Shape::new_sphere(2.0),
            Vector3::new(5.0, 0.0, 0.0),
            Vector4::new(0.0, 0.0, 0.0, 1.0),
        ),
    ]);
    let mass = 0.1;

    dynamics_world.add_rigid_body(RigidBody::new(
        mass,
        shape.calculate_local_inertia(mass),
        shape,
        Vector3::new(4.0, 2.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));

    for _ in 0..10 {
        dynamics_world.step_simulation(0.1, 0, 0.0);
    }
}

#[test()]
fn new_convex_hull() {
    Shape::new_convex_hull(&vec![
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(0.7235999703407288, -0.5257200002670288, -0.4472149908542633),
        Vector3::new(-0.27638500928878784, -0.8506399989128113, -0.4472149908542633),
        Vector3::new(-0.8944249749183655, 0.0, -0.4472149908542633),
        Vector3::new(-0.27638500928878784, 0.8506399989128113, -0.4472149908542633),
        Vector3::new(0.7235999703407288, 0.5257200002670288, -0.4472149908542633),
        Vector3::new(0.27638500928878784, -0.8506399989128113, 0.4472149908542633),
        Vector3::new(-0.7235999703407288, -0.5257200002670288, 0.4472149908542633),
        Vector3::new(-0.7235999703407288, 0.5257200002670288, 0.4472149908542633),
        Vector3::new(0.27638500928878784, 0.8506399989128113, 0.4472149908542633),
        Vector3::new(0.8944249749183655, 0.0, 0.4472149908542633),
        Vector3::new(0.0, 0.0, 1.0),
    ]);
}
