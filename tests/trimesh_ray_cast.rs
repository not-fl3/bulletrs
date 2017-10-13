extern crate bulletrs;

use bulletrs::*;

#[test(trimesh_raycast)]
fn trimesh_raycast() {
    let bullet = Bullet::connect(ConnectMethod::Direct).unwrap();
    let client = bullet.physics_client_handle();
    client.reset_simulation();

    let plane_shape = client
        .create_collision_shape(ShapeType::Plane {
            normal: Vector3::from([0.0, 0.0, 1.0]),
            constant: 0.0,
        })
        .unwrap();

    let plane = client
        .create_rigid_body(
            plane_shape,
            0.0,
            Vector3::from([0.0, 0.0, 0.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();

    client.change_dynamics_info(
        plane,
        DynamicsInfo {
            restitution: Some(0.9),
            ..Default::default()
        },
    );


    let icosphere : Vec<Vector3<f64>> = vec![
        Vector3::from([0.0, 0.0, -1.0]),
        Vector3::from([0.7235999703407288, -0.5257200002670288, -0.4472149908542633]),
        Vector3::from([-0.27638500928878784, -0.8506399989128113, -0.4472149908542633]),
        Vector3::from([-0.8944249749183655, 0.0, -0.4472149908542633]),
        Vector3::from([-0.27638500928878784, 0.8506399989128113, -0.4472149908542633]),
        Vector3::from([0.7235999703407288, 0.5257200002670288, -0.4472149908542633]),
        Vector3::from([0.27638500928878784, -0.8506399989128113, 0.4472149908542633]),
        Vector3::from([-0.7235999703407288, -0.5257200002670288, 0.4472149908542633]),
        Vector3::from([-0.7235999703407288, 0.5257200002670288, 0.4472149908542633]),
        Vector3::from([0.27638500928878784, 0.8506399989128113, 0.4472149908542633]),
        Vector3::from([0.8944249749183655, 0.0, 0.4472149908542633]),
        Vector3::from([0.0, 0.0, 1.0]),
    ];
    let shape = client
        .create_collision_shape(ShapeType::TriMesh {
            vertices: icosphere,
            scale: Vector3::from([1.0, 1.0, 1.0]),
        })
        .unwrap();

    let body = client
        .create_rigid_body(
            shape,
            0.1,
            Vector3::from([0.0, 0.0, 4.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();
    body.set_user_data(Box::new(123));

    client.change_dynamics_info(
        body,
        DynamicsInfo {
            restitution: Some(0.9),
            ..Default::default()
        },
    );


    for _ in 0 .. 1000 {
        let results = client
            .raycast(
                Point3::from([-20.0, 0.0, 4.0]),
                Point3::from([20.0, 0.0, 4.0]),
            )
            .unwrap();

        client.step_simulation();

        assert_eq!(results.len(), 1);

        let mut tois: Vec<f64> = results
            .iter()
            .map(|collision| collision.fraction * 40.0)
            .collect();
        tois.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let user_data = results[0].body.as_ref().unwrap().get_user_data::<i32>().unwrap();
        assert_eq!(*user_data, 123);
    }
}
