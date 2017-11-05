extern crate bulletrs;
extern crate cgmath;

use cgmath::{Vector3, Vector4};

use bulletrs::*;

#[test()]
fn ray_test() {
    let configuration = CollisionConfiguration::new_default();

    let mut dynamics_world = DynamicsWorld::new_discrete_world(
        CollisionDispatcher::new(&configuration),
        Broadphase::new(BroadphaseInterface::DbvtBroadphase),
        ConstraintSolver::new(),
        configuration,
    );
    dynamics_world.set_gravity(Vector3::new(0.0, 0.0, 0.0));

    let shape = Shape::new_sphere(1.0);
    let mass = 0.1;
    let body1 = RigidBody::new(
        mass,
        shape.calculate_local_inertia(mass),
        shape,
        Vector3::new(-4.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    );
    dynamics_world.add_rigid_body(body1);

    let shape2 = Shape::new_sphere(1.0);
    let mass = 0.1;
    let body2 = RigidBody::new(
        mass,
        shape2.calculate_local_inertia(mass),
        shape2,
        Vector3::new(4.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    );
    dynamics_world.add_rigid_body(body2);

    for _ in 0 .. 500 {
        dynamics_world.step_simulation(0.1, 5, 1.0 / 60.0);
    }

    let result = dynamics_world.raytest(ClosestRayResultCallback::new(
        Vector3::new(-10.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
    ));

    assert_eq!(result.closest_hit_fraction(), 0.25);
    assert_eq!(result.intersections().len(), 1);

    let result = dynamics_world.raytest(AllRayResultCallback::new(
        Vector3::new(-10.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
    ));

    assert_eq!(result.intersections().len(), 2);

    for result in result.intersections() {
        assert_eq!(result.rigidbody().as_ref().unwrap().removed(), false);
        dynamics_world.remove_body(result.rigidbody().as_ref().unwrap());
        assert_eq!(result.rigidbody().as_ref().unwrap().removed(), true);
    }
}

