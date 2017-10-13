extern crate bulletrs;

use bulletrs::*;

#[test]
fn raycast_test() {
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

    for _ in 0 .. 1000 {
        let results = client
            .raycast(
                Point3::from([-20.0, 0.0, 0.0]),
                Point3::from([20.0, 0.0, 0.0]),
            )
            .unwrap();

        client.step_simulation();

        assert_eq!(results.len(), 3);
        let mut tois: Vec<f64> = results
            .iter()
            .map(|collision| collision.fraction * 40.0)
            .collect();
        tois.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(tois[0], 15.0);
        assert_eq!(tois[1], 19.0);
        assert_eq!(tois[2], 23.0);
    }
}
