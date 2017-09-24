#![feature(test)]

extern crate test;
extern crate bulletrs;

use test::Bencher;

use bulletrs::*;

#[bench]
fn get_base_position_and_orientation_bench(b: &mut Bencher) {
    let bodies_count = 2000;

    let bullet = Bullet::connect(ConnectMethod::Direct).unwrap();
    let client = bullet.physics_client_handle();

    client.reset_simulation();
    client.set_gravity(0.0, 0.0, -10.0);
    client.set_realtime_simulation(false);
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
            restitution : Some(0.9),
            ..Default::default()
        },
        );

    let sphere_shape = client
        .create_collision_shape(ShapeType::Sphere { radius: 0.1 })
        .unwrap();

    let bodies = (0..bodies_count).map(|i| {
        let body = client
            .create_rigid_body(
                sphere_shape.clone(),
                0.1,
                Vector3::from([i as f64 / 1000.0, i as f64 / 1000.0, 2.0 + i as f64 / 10.0]),
                Vector4::from([0.0, 0.0, 0.0, 1.0]),
            )
            .unwrap();

        client.change_dynamics_info(
            body.clone(),
            DynamicsInfo {
                restitution : Some(0.9),
                ..Default::default()
            },
        );
        body.clone()
    }).collect::<Vec<_>>();

    b.iter(move || {
        for body in bodies.iter() {
            client.get_base_position_and_orientation(body.clone()).unwrap();
        }
    })
}
