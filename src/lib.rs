extern crate bulletrs_sys as sys;
extern crate mint;

mod bullet;
mod errors;
mod physics_client;
mod command;
mod status;
mod shape;
mod multibody;

pub use bullet::{Bullet, ConnectMethod};
pub use physics_client::{ShapeType};
pub use multibody::DynamicsInfo;
pub use mint::{Vector3, Vector4};
