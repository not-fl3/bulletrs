extern crate bulletrs;

use bulletrs::*;

#[test]
fn get_actual_state() {
    let bullet = Bullet::connect(ConnectMethod::Direct).unwrap();
    let client = bullet.physics_client_handle();

    client.reset_simulation();

    let sphere_shape = client
        .create_collision_shape(ShapeType::Sphere { radius: 0.1 })
        .unwrap();

    let body = client
        .create_rigid_body(
            sphere_shape.clone(),
            0.1,
            Vector3::from([1.0, 2.0, 3.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();

    let (position, _) = body.get_base_position_and_orientation().unwrap();
    let velocity = body.get_linear_velocity().unwrap();

    assert_eq!(position, Vector3::from([1.0, 2.0, 3.0]));
    assert_eq!(velocity, Vector3::from([0.0, 0.0, 0.0]));
}

#[test]
fn reset_base_velocity() {
    let bullet = Bullet::connect(ConnectMethod::Direct).unwrap();
    let client = bullet.physics_client_handle();

    client.reset_simulation();

    let sphere_shape = client
        .create_collision_shape(ShapeType::Sphere { radius: 0.1 })
        .unwrap();

    let body = client
        .create_rigid_body(
            sphere_shape.clone(),
            0.1,
            Vector3::from([1.0, 2.0, 3.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();

    let velocity = body.get_linear_velocity().unwrap();
    assert_eq!(velocity, Vector3::from([0.0, 0.0, 0.0]));

    body.reset_base_velocity(Some(Vector3::from([1.0,2.0,3.0])),None).unwrap();
    let new_velocity = body.get_linear_velocity().unwrap();
    assert_eq!(new_velocity, Vector3::from([1.0, 2.0, 3.0]));

}
