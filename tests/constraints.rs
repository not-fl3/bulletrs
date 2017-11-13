extern crate bulletrs;
extern crate cgmath;

use cgmath::{Vector3, Vector4};

use bulletrs::*;

#[test()]
fn hinge() {
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

    let body_a = dynamics_world.add_rigid_body(RigidBody::new(
        0.0,
        Vector3::new(0.0, 0.0, 0.0),
        shape,
        Vector3::new(4.0, 2.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));

    let shape = Shape::new_sphere(2.0);
    let mass = 1.0;
    let body_b = dynamics_world.add_rigid_body(RigidBody::new(
        mass,
        shape.calculate_local_inertia(mass),
        shape,
        Vector3::new(4.0, 2.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));



    let hinge = HingeConstraint::new(
        &body_b,
        &body_a,
        Vector3::from([3.0, 3.0, 3.0]),
        Vector3::from([0.0, 0.0, 0.0]),
        Vector3::from([1.0, 0.0, 0.0]),
        Vector3::from([1.0, 0.0, 0.0]),
        false
    );
    dynamics_world.add_constraint(hinge, true);

    for _ in 0..1000 {
        let (_position, _) = body_b.get_world_position_and_orientation();

        //println!("{}", _position.y);

        dynamics_world.step_simulation(0.1, 0, 0.0);
    }
    let (position, _) = body_b.get_world_position_and_orientation();

    assert!(position.y >= -100.0);

}
