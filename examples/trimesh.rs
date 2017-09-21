extern crate bulletrs;

use bulletrs::*;

fn main() {
    let bullet = Bullet::connect(ConnectMethod::Gui).unwrap();
    let client = bullet.physics_client_handle();

    client.reset_simulation();
    client.set_gravity(0.0, 0.0, -10.0);
    client.set_realtime_simulation(true);

    let plane_shape = client
        .create_collision_shape(ShapeType::Plane {
            normal: Vector3::from([0.0, 0.0, 1.0]),
            constant: 0.0,
        })
        .unwrap();

    let plane = client
        .create_multi_body(
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
        .create_multi_body(
            shape,
            0.1,
            Vector3::from([0.0, 0.0, 4.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();

    client.change_dynamics_info(
        body,
        DynamicsInfo {
            restitution: Some(0.9),
            ..Default::default()
        },
    );


    loop {
        client.step_simulation();
        ::std::thread::sleep(::std::time::Duration::from_millis(30));
    }
}
