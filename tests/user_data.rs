extern crate bulletrs;

use bulletrs::*;

#[test]
fn body_user_data() {
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
            Vector3::from([1.0, 2.0, 3.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();
    body1.set_user_data(Box::new(1));

    let body2 = client
        .create_rigid_body(
            sphere_shape.clone(),
            0.1,
            Vector3::from([1.0, 2.0, 3.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();
    body2.set_user_data(Box::new(2));

    for _ in 0 .. 10 {
        client.step_simulation();
    }

    assert_eq!(1, *body1.get_user_data::<i32>().unwrap());
    assert_eq!(2, *body2.get_user_data::<i32>().unwrap());
}
