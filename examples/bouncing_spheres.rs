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
            restitution : Some(0.9),
            ..Default::default()
        },
        );

    let sphere_shape = client
        .create_collision_shape(ShapeType::Sphere { radius: 0.1 })
        .unwrap();

    for i in 0..20 {
        let body = client
            .create_multi_body(
                sphere_shape,
                0.1,
                Vector3::from([i as f64 / 1000.0, i as f64 / 1000.0, 2.0 + i as f64 / 10.0]),
                Vector4::from([0.0, 0.0, 0.0, 1.0]),
            )
            .unwrap();

        client.change_dynamics_info(
            body,
            DynamicsInfo {
                restitution : Some(0.9),
                ..Default::default()
            },
        );
    }

    loop {
        client.step_simulateion();
        ::std::thread::sleep(::std::time::Duration::from_millis(30));
    }
}
