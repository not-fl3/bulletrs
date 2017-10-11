extern crate bulletrs;

use bulletrs::*;

#[test]
fn get_actual_state() {
    let bullet = Bullet::connect(ConnectMethod::Direct).unwrap();
    let client = bullet.physics_client_handle();

    client.reset_simulation();

    let sphere_shape = client
        .create_collision_shape(ShapeType::Sphere { radius: 1.0 })
        .unwrap();

    let _ = client
        .create_rigid_body(
            sphere_shape.clone(),
            0.1,
            Vector3::from([0.0, 0.0, 0.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();

    let _ = client
        .create_rigid_body(
            sphere_shape.clone(),
            0.1,
            Vector3::from([4.0, 0.0, 0.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();

    let _ = client
        .create_rigid_body(
            sphere_shape.clone(),
            0.1,
            Vector3::from([-4.0, 0.0, 0.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();

    let results = client.raycast(
        Point3::from([-20.0, 0.0, 0.0]),
        Point3::from([20.0, 0.0, 0.0]),
    ).unwrap();

    assert_eq!(results.len(), 3);
}
