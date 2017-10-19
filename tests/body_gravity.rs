extern crate bulletrs;

use bulletrs::*;

#[test]
fn get_actual_state() {
    let bullet = Bullet::connect(ConnectMethod::Direct).unwrap();
    let client = bullet.physics_client_handle();
    client.set_gravity(0.0, 0.0, -10.0);

    client.reset_simulation();

    let sphere_shape = client
        .create_collision_shape(ShapeType::Sphere { radius: 0.1 })
        .unwrap();

    let body1 = client
        .create_rigid_body(
            sphere_shape.clone(),
            0.1,
            Vector3::from([5.0, 2.0, 3.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();
    let body2 = client
        .create_rigid_body(
            sphere_shape.clone(),
            0.1,
            Vector3::from([1.0, 2.0, 3.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();
    body2.set_body_gravity(Vector3::from([0.0, 0.0, 0.0])).unwrap();
    for _ in 0..10 {
        client.step_simulation();
    }

    let (position, _) = body1.get_base_position_and_orientation().unwrap();
    assert!(position != Vector3::from([5.0, 2.0, 3.0]));

    let (position, _) = body2.get_base_position_and_orientation().unwrap();
    assert_eq!(position, Vector3::from([1.0, 2.0, 3.0]));

}
