extern crate bulletrs;
extern crate cgmath;

use cgmath::{Vector3, Vector4};

use bulletrs::*;


fn main() {
    let configuration = CollisionConfiguration::new_default();

    let dynamics_world = DynamicsWorld::new_discrete_world(
        CollisionDispatcher::new(&configuration),
        Broadphase::new(BroadphaseInterface::DbvtBroadphase),
        ConstraintSolver::new(),
        configuration,
    );

    dynamics_world.set_gravity(Vector3::new(0.0, -10.0, 0.0));

    let fall_shape = Shape::new_sphere(2.0);
    let mass = 1.0;
    let fall_inertia = fall_shape.calculate_local_inertia(mass);
    let fall_rigid_body = RigidBody::new(
        mass,
        fall_shape,
        fall_inertia,
        Vector3::new(0.0, 5.0, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    );
    dynamics_world.add_rigid_body(&fall_rigid_body);

    for _ in 0 .. 100 {
        dynamics_world.step(0.01, 0, 0.01);
        println!("{:?}", fall_rigid_body.get_world_transform());
    }
}
