extern crate bulletrs;

use bulletrs::*;

fn main() {
    let bullet = Bullet::connect(ConnectMethod::Gui).unwrap();
    let client = bullet.physics_client_handle();
    client.reset_simulation();

    client.add_user_debug_line(
        Point3::from([0.0, 0.0, 0.0]),
        Point3::from([0.0, 0.0, 10.0]),
        Vector4::from([1.0, 0.0, 0.0, 1.0]),
    );
    client.add_user_debug_line(
        Point3::from([0.0, 0.0, 0.0]),
        Point3::from([0.0, 3.0, 10.0]),
        Vector4::from([0.0, 1.0, 0.0, 1.0]),
    );
    client.add_user_debug_line(
        Point3::from([0.0, 0.0, 0.0]),
        Point3::from([0.0, -3.0, 10.0]),
        Vector4::from([0.0, 0.0, 1.0, 1.0]),
    );
    loop {
        client.step_simulation();

        ::std::thread::sleep(::std::time::Duration::from_millis(30));
    }
}
