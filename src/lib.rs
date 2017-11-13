extern crate mint;
pub extern crate bulletrs_sys as sys;

mod collision;
mod dynamics;
pub(crate) mod bullet_vector3;

mod errors;

pub use mint::{Point3, Vector3, Vector4};
pub use errors::Error;

pub use collision::broadphase_collision::{Broadphase, BroadphaseInterface};
pub use collision::collision_dispatch::{CollisionConfiguration, CollisionDispatcher};
pub use collision::collision_shapes::{CapsuleAxis, Shape};
pub use dynamics::constraint_solver::{ConstraintSolver, HingeConstraint};
pub use dynamics::dynamics_world::{AllRayResultCallback, ClosestRayResultCallback, DynamicsWorld,
                                   RayResultCallback};
pub use dynamics::rigid_body::{RigidBody, RigidBodyHandle};

pub(crate) use bullet_vector3::BulletVector3;
