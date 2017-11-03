extern crate bulletrs;
extern crate cgmath;

use cgmath::{Vector3, Vector4};

use bulletrs::*;

#[test()]
fn set_get_user_data() {
    let configuration = CollisionConfiguration::new_default();

    let mut dynamics_world = DynamicsWorld::new_discrete_world(
        CollisionDispatcher::new(&configuration),
        Broadphase::new(BroadphaseInterface::DbvtBroadphase),
        ConstraintSolver::new(),
        configuration,
    );

    let shape = Shape::new_sphere(1.0);
    let mass = 0.1;
    let body1 = dynamics_world.add_rigid_body(RigidBody::new(
        mass,
        shape.calculate_local_inertia(mass),
        shape,
        Vector3::new(-4.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));
    body1.set_user_data(5);
    assert_eq!(*unsafe { body1.get_user_data::<i32>().unwrap() }, 5);
}

#[test()]
fn ray_test_user_data() {
    let configuration = CollisionConfiguration::new_default();

    let mut dynamics_world = DynamicsWorld::new_discrete_world(
        CollisionDispatcher::new(&configuration),
        Broadphase::new(BroadphaseInterface::DbvtBroadphase),
        ConstraintSolver::new(),
        configuration,
    );

    let shape = Shape::new_sphere(1.0);
    let mass = 0.1;
    let body1 = dynamics_world.add_rigid_body(RigidBody::new(
        mass,
        shape.calculate_local_inertia(mass),
        shape,
        Vector3::new(-4.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));
    body1.set_user_data(5);

    let shape2 = Shape::new_sphere(2.0);
    let mass = 0.1;
    let body2 = dynamics_world.add_rigid_body(RigidBody::new(
        mass,
        shape2.calculate_local_inertia(mass),
        shape2,
        Vector3::new(4.0, 0.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    ));
    body2.set_user_data(8);

    let result = dynamics_world.raytest(ClosestRayResultCallback::new(
        Vector3::new(-10.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
    ));

    assert_eq!(result.closest_hit_fraction(), 0.25);
    assert_eq!(result.intersections().len(), 1);
    let body = result.intersections()[0].rigidbody();
    assert!(body.is_some());
    assert_eq!(*unsafe { body.unwrap().get_user_data::<i32>().unwrap() }, 5);

    let result = dynamics_world.raytest(AllRayResultCallback::new(
        Vector3::new(-10.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
    ));

    assert_eq!(result.intersections().len(), 2);
}
