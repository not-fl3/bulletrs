extern crate bulletrs;
extern crate cgmath;

use cgmath::{Vector3, Vector4};

use bulletrs::*;

#[test()]
fn compound_mesh() {
    let configuration = CollisionConfiguration::new_default();

    let dynamics_world = DynamicsWorld::new_discrete_world(
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
    let body1 = RigidBody::new(
        mass,
        shape.calculate_local_inertia(mass),
        shape,
        Vector3::new(4.0, 2.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    );
    dynamics_world.add_rigid_body(&body1);

    for _ in 0..10 {
        dynamics_world.step(0.1, 0, 0.0);
    }
}
