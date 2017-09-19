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

    let shape = client
        .create_collision_shape(ShapeType::Compound(vec![
            (
                ShapeType::Sphere { radius: 0.1 },
                Vector3::from([0.0, -0.1, 0.0]),
                Vector4::from([0.0, 0.0, 0.0, 1.0]),
            ),
            (
                ShapeType::Sphere { radius: 0.1 },
                Vector3::from([0.0, 0.1, 0.0]),
                Vector4::from([0.0, 0.0, 0.0, 1.0]),
            ),
        ]))
        .unwrap();

    let compound = client
        .create_multi_body(
            shape,
            0.1,
            Vector3::from([0.0, 0.0, 2.0]),
            Vector4::from([0.0, 0.0, 0.0, 1.0]),
        )
        .unwrap();

    client.change_dynamics_info(
        compound,
        DynamicsInfo {
            restitution: Some(0.9),
            ..Default::default()
        },
    );


    loop {
        client.step_simulateion();
        ::std::thread::sleep(::std::time::Duration::from_millis(30));
    }
}
