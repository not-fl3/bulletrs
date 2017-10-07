extern crate bulletrs;

use bulletrs::*;

fn main() {
    let bullet = Bullet::connect(ConnectMethod::Direct).unwrap();
    let client = bullet.physics_client_handle();
    client.set_gravity(0.0, 0.0, -10.0);

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

    loop {
        client.step_simulation();

        let (position, _) = body.get_base_position_and_orientation().unwrap();

        println!("{:?}", position);
    }
}
