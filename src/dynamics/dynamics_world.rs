use sys;

use collision::broadphase_collision::Broadphase;
use collision::collision_dispatch::{CollisionConfiguration, CollisionDispatcher};
use dynamics::constraint_solver::ConstraintSolver;
use dynamics::rigid_body::RigidBody;
use bullet_vector3::BulletVector3;
use mint::Vector3;

pub enum DynamicsWorld {
    Discrete {
        world: sys::btDiscreteDynamicsWorld,
        data: (
            Box<CollisionDispatcher>,
            Box<Broadphase>,
            Box<ConstraintSolver>,
            Box<CollisionConfiguration>,
        ),
    },
}

impl DynamicsWorld {
    pub fn new_discrete_world(
        dispatcher: CollisionDispatcher,
        broadphase: Broadphase,
        solver: ConstraintSolver,
        configuration: CollisionConfiguration,
    ) -> Self {
        let dispatcher_box = Box::new(dispatcher);
        let broadphase_box = Box::new(broadphase);
        let solver_box = Box::new(solver);
        let configuration_box = Box::new(configuration);
        DynamicsWorld::Discrete {
            world: unsafe {
                sys::btDiscreteDynamicsWorld::new(
                    dispatcher_box.as_ptr(),
                    broadphase_box.as_ptr(),
                    solver_box.as_ptr(),
                    configuration_box.as_ptr(),
                )
            },
            data: (
                dispatcher_box,
                broadphase_box,
                solver_box,
                configuration_box,
            ),
        }
    }

    pub fn set_gravity<T: Into<Vector3<f64>>>(&self, gravity: T) {
        let gravity: BulletVector3 = gravity.into().into();
        match self {
            &DynamicsWorld::Discrete { ref world, .. } => unsafe {
                sys::btDiscreteDynamicsWorld_setGravity(
                    world as *const _ as *mut _,
                    &gravity.0 as *const _ as *const _,
                );
            },
        }
    }

    pub fn add_rigid_body(&self, rigid_body: &RigidBody) {
        match self {
            &DynamicsWorld::Discrete { ref world, .. } => unsafe {
                sys::btDiscreteDynamicsWorld_addRigidBody(
                    world as *const _ as *mut _,
                    rigid_body.as_ptr(),
                );
            },
        }
    }

    pub fn step(&self, time_step: f64, max_sub_steps: i32, fixed_time_step: f64) {
        match self {
            &DynamicsWorld::Discrete { ref world, .. } => unsafe {
                sys::btDiscreteDynamicsWorld_stepSimulation(
                    world as *const _ as *mut _,
                    time_step,
                    max_sub_steps,
                    fixed_time_step,
                );
            }
        }
    }
}
