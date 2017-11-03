use sys;

use collision::broadphase_collision::Broadphase;
use collision::collision_dispatch::{CollisionConfiguration, CollisionDispatcher};
use dynamics::constraint_solver::ConstraintSolver;
use dynamics::rigid_body::{RigidBody, RigidBodyHandle};
use bullet_vector3::BulletVector3;
use mint::Vector3;

/// Internal data storage, owning all rigidbodys of the world
pub struct InternalWorldData {
    rigid_bodys: Vec<RigidBody>,
}
impl InternalWorldData {
    pub fn new() -> Self {
        InternalWorldData {
            rigid_bodys: vec![],
        }
    }
}

pub enum WorldImplementation {
    Discrete {
        world: sys::btDiscreteDynamicsWorld,
        init_data: (
            Box<CollisionDispatcher>,
            Box<Broadphase>,
            Box<ConstraintSolver>,
            Box<CollisionConfiguration>,
        ),
    },
}

pub struct DynamicsWorld {
    implementation: WorldImplementation,
    world_data: InternalWorldData,
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
        DynamicsWorld {
            implementation: WorldImplementation::Discrete {
                world: unsafe {
                    sys::btDiscreteDynamicsWorld::new(
                        dispatcher_box.as_ptr(),
                        broadphase_box.as_ptr(),
                        solver_box.as_ptr(),
                        configuration_box.as_ptr(),
                    )
                },
                init_data: (
                    dispatcher_box,
                    broadphase_box,
                    solver_box,
                    configuration_box,
                ),
            },
            world_data: InternalWorldData::new(),
        }
    }

    pub fn set_gravity<T: Into<Vector3<f64>>>(&self, gravity: T) {
        let gravity: BulletVector3 = gravity.into().into();
        match &self.implementation {
            &WorldImplementation::Discrete { ref world, .. } => unsafe {
                sys::btDiscreteDynamicsWorld_setGravity(
                    world as *const _ as *mut _,
                    &gravity.0 as *const _ as *const _,
                );
            },
        }
    }

    pub fn add_rigid_body(&mut self, rigid_body: RigidBody) -> RigidBodyHandle {
        self.world_data.rigid_bodys.push(rigid_body);
        let added_element = self.world_data.rigid_bodys.last().unwrap();
        match &self.implementation {
            &WorldImplementation::Discrete { ref world, .. } => unsafe {
                sys::btDiscreteDynamicsWorld_addRigidBody(
                    world as *const _ as *mut _,
                    added_element.as_ptr(),
                );
                RigidBodyHandle::new(
                    added_element.as_ptr(),
                    added_element.motion_state_ptr()
                )
            },
        }
    }

    pub fn remove_body(&mut self, rigid_body: &RigidBodyHandle) {
        match &self.implementation {
            &WorldImplementation::Discrete { ref world, .. } => unsafe {

                sys::btDiscreteDynamicsWorld_removeRigidBody(world as *const _ as *mut _,
                                                             rigid_body.ptr)
            }
        }
    }

    pub fn step_simulation(&self, time_step: f64, max_sub_steps: i32, fixed_time_step: f64) {
        match &self.implementation {
            &WorldImplementation::Discrete { ref world, .. } => unsafe {
                sys::btDiscreteDynamicsWorld_stepSimulation(
                    world as *const _ as *mut _,
                    time_step,
                    max_sub_steps,
                    fixed_time_step,
                );
            },
        }
    }

    pub fn raytest<C>(&self, mut callback: C) -> C
    where
        C: RayResultCallback + InternalRayResultCallback,
    {
        let from = callback.world_from();
        let to = callback.world_to();
        match &self.implementation {
            &WorldImplementation::Discrete { ref world, .. } => unsafe {
                sys::btCollisionWorld_rayTest(
                    world as *const _ as *mut _,
                    &from as *const _,
                    &to as *const _,
                    callback.as_ptr(),
                )
            },
        }
        callback
    }
}

pub struct RayIntersection {
    pub fraction: f64,
    pub normal: Vector3<f64>,
    pub point: Vector3<f64>,

    collision_object: *const sys::btCollisionObject,
}

impl RayIntersection {
    pub fn rigidbody(&self) -> Option<RigidBodyHandle> {
        if self.collision_object.is_null() {
            return None;
        } else {
            let rigid_body = self.collision_object as *mut sys::btRigidBody;
            let motion_state = unsafe { &(*rigid_body).m_optionalMotionState as *const _ as *mut _};
            Some(RigidBodyHandle::new(rigid_body, motion_state, ))

        }
    }
}
/// Internal and unsafe methods.
/// Not exported so can't be imported and used.
pub trait InternalRayResultCallback {
    fn world_from(&self) -> sys::btVector3;
    fn world_to(&self) -> sys::btVector3;
    fn as_ptr(&mut self) -> *mut sys::btCollisionWorld_RayResultCallback;
}

pub trait RayResultCallback {
    fn intersections(&self) -> Vec<RayIntersection>;
}

pub struct AllRayResultCallback {
    callback: sys::btCollisionWorld_AllHitsRayResultCallback,
}

impl AllRayResultCallback {
    pub fn new<T, T1>(from: T, to: T1) -> Self
    where
        T: Into<Vector3<f64>>,
        T1: Into<Vector3<f64>>,
    {
        let from: BulletVector3 = from.into().into();
        let to: BulletVector3 = to.into().into();

        AllRayResultCallback {
            callback: unsafe {
                sys::btCollisionWorld_AllHitsRayResultCallback::new(
                    &from.0 as *const _ as *const _,
                    &to.0 as *const _ as *const _,
                )
            },
        }
    }
}

impl InternalRayResultCallback for AllRayResultCallback {
    fn world_from(&self) -> sys::btVector3 {
        self.callback.m_rayFromWorld.clone()
    }
    fn world_to(&self) -> sys::btVector3 {
        self.callback.m_rayToWorld.clone()
    }

    fn as_ptr(&mut self) -> *mut sys::btCollisionWorld_RayResultCallback {
        &mut self.callback as *mut _ as *mut _
    }
}

impl RayResultCallback for AllRayResultCallback {
    fn intersections(&self) -> Vec<RayIntersection> {
        let normals = unsafe {
            ::std::slice::from_raw_parts(
                self.callback.m_hitNormalWorld.m_data,
                self.callback.m_hitNormalWorld.m_size as usize,
            )
        };
        let points = unsafe {
            ::std::slice::from_raw_parts(
                self.callback.m_hitPointWorld.m_data,
                self.callback.m_hitPointWorld.m_size as usize,
            )
        };
        let fractions = unsafe {
            ::std::slice::from_raw_parts(
                self.callback.m_hitFractions.m_data,
                self.callback.m_hitFractions.m_size as usize,
            )
        };
        let objects = unsafe {
            ::std::slice::from_raw_parts(
                self.callback.m_collisionObjects.m_data,
                self.callback.m_collisionObjects.m_size as usize,
            )
        };

        let mut intersections = vec![];

        for i in 0..normals.len() {
            intersections.push(RayIntersection {
                collision_object: objects[i],
                fraction: fractions[i],
                normal: Vector3::from_slice(&normals[i].m_floats[0..3]),
                point: Vector3::from_slice(&points[i].m_floats[0..3]),
            });
        }
        intersections
    }
}

pub struct ClosestRayResultCallback {
    callback: sys::btCollisionWorld_ClosestRayResultCallback,
}


impl ClosestRayResultCallback {
    pub fn new<T, T1>(from: T, to: T1) -> Self
    where
        T: Into<Vector3<f64>>,
        T1: Into<Vector3<f64>>,
    {
        let from: BulletVector3 = from.into().into();
        let to: BulletVector3 = to.into().into();

        ClosestRayResultCallback {
            callback: unsafe {
                sys::btCollisionWorld_ClosestRayResultCallback::new(
                    &from.0 as *const _ as *const _,
                    &to.0 as *const _ as *const _,
                )
            },
        }
    }

    pub fn closest_hit_fraction(&self) -> f64 {
        self.callback._base.m_closestHitFraction
    }
}

impl InternalRayResultCallback for ClosestRayResultCallback {
    fn world_from(&self) -> sys::btVector3 {
        self.callback.m_rayFromWorld.clone()
    }
    fn world_to(&self) -> sys::btVector3 {
        self.callback.m_rayToWorld.clone()
    }
    fn as_ptr(&mut self) -> *mut sys::btCollisionWorld_RayResultCallback {
        &mut self.callback as *mut _ as *mut _
    }
}

impl RayResultCallback for ClosestRayResultCallback {
    fn intersections(&self) -> Vec<RayIntersection> {
        vec![
            RayIntersection {
                collision_object: self.callback._base.m_collisionObject,
                fraction: self.callback._base.m_closestHitFraction,
                point: Vector3::from_slice(&self.callback.m_hitPointWorld.m_floats[0..3]),
                normal: Vector3::from_slice(&self.callback.m_hitNormalWorld.m_floats[0..3]),
            },
        ]
    }
}
