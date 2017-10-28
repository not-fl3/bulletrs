extern crate bulletrs;

use bulletrs::sys::bt_bullet_dynamics_common as bt;
use bulletrs::sys::rust_helpers::*;

#[test]
fn bindged_generated_test() {
    unsafe {
        let mut broadphase = bt::btDbvtBroadphase::new(std::ptr::null_mut());
        let info = bt::btDefaultCollisionConstructionInfo {
            m_persistentManifoldPool: std::ptr::null_mut(),
            m_collisionAlgorithmPool: std::ptr::null_mut(),
            m_defaultMaxPersistentManifoldPoolSize: 4096,
            m_defaultMaxCollisionAlgorithmPoolSize: 4096,
            m_customCollisionAlgorithmMaxElementSize: 0,
            m_useEpaPenetrationAlgorithm: 1,
        };
        let mut collision_configuration =
            bt::btDefaultCollisionConfiguration::new(&info as *const _);
        let mut dispatcher =
            bt::btCollisionDispatcher::new(&mut collision_configuration as *mut _ as *mut _);
        let mut solver = bt::btSequentialImpulseConstraintSolver::new();
        let mut dynamics_world = bt::btDiscreteDynamicsWorld::new(
            &mut dispatcher as *mut _ as *mut _,
            &mut broadphase as *mut _ as *mut _,
            &mut solver as *mut _ as *mut _,
            &mut collision_configuration as *mut _ as *mut _,
        );

        let gravity = bt::btVector3 {
            m_floats: [0.0, -10.0, 0.0, 0.0],
        };
        bt::btDiscreteDynamicsWorld_setGravity(
            &mut dynamics_world as *mut _ as *mut _,
            &gravity as *const _ as *const _,
        );


        let mut fall_shape = newSphereShape(2.0);
        let sphere_rotation = bt::btMatrix3x3 {
            m_el: [
                bt::btVector3 {
                    m_floats: [1.0, 0.0, 0.0, 0.0],
                },
                bt::btVector3 {
                    m_floats: [0.0, 1.0, 0.0, 0.0],
                },
                bt::btVector3 {
                    m_floats: [0.0, 0.0, 1.0, 0.0],
                },
            ],
        };
        let sphere_transform = bt::btTransform {
            m_basis: sphere_rotation.clone(),
            m_origin: bt::btVector3 {
                m_floats: [0.0, 5.0, 0.0, 0.0],
            },
        };

        let mut fall_motion_state = newDefaultMotionState(&sphere_transform as *const _);

        let mass = 1.0;
        let mut fall_inertia = bt::btVector3 {
            m_floats: [0.0, 0.0, 0.0, 0.0],
        };
        bt::btSphereShape_calculateLocalInertia(
            &mut fall_shape as *mut _ as *mut _,
            mass,
            &mut fall_inertia as *mut _,
        );

        let world_rotation = bt::btMatrix3x3 {
            m_el: [
                bt::btVector3 {
                    m_floats: [1.0, 0.0, 0.0, 0.0],
                },
                bt::btVector3 {
                    m_floats: [0.0, 1.0, 0.0, 0.0],
                },
                bt::btVector3 {
                    m_floats: [0.0, 0.0, 1.0, 0.0],
                },
            ],
        };
        let world_transform = bt::btTransform {
            m_basis: world_rotation.clone(),
            m_origin: bt::btVector3 {
                m_floats: [0.0, 0.0, 0.0, 0.0],
            },
        };

        let mut fall_rigid_body_ci = bt::btRigidBody_btRigidBodyConstructionInfo {
            m_mass: mass,
            m_motionState: &mut fall_motion_state as *mut _ as *mut _,
            m_collisionShape: &mut fall_shape as *mut _ as *mut _,
            m_localInertia: fall_inertia,
            m_linearDamping: 0.0,
            m_angularDamping: 0.0,
            m_friction: 0.5,
            m_rollingFriction: 0.0,
            m_spinningFriction: 0.0,
            m_restitution: 0.0,
            m_linearSleepingThreshold: 0.8,
            m_angularSleepingThreshold: 1.0,
            m_additionalDamping: false,
            m_additionalDampingFactor: 0.005,
            m_additionalLinearDampingThresholdSqr: 0.01,
            m_additionalAngularDampingThresholdSqr: 0.01,
            m_additionalAngularDampingFactor: 0.01,
            m_startWorldTransform: world_transform
        };
        let mut fall_rigid_body = bt::btRigidBody::new(&mut fall_rigid_body_ci as *mut _ as *mut _);
        bt::btDiscreteDynamicsWorld_addRigidBody(&mut dynamics_world as *mut _ as *mut _, &mut fall_rigid_body as *mut _ as *mut _);

        let mut old_y = 5.0;
        for _ in 0 .. 100 {
            bt::btDiscreteDynamicsWorld_stepSimulation(&mut dynamics_world as *mut _ as *mut _, 0.01, 0, 0.01);
            let motion_state : *mut bt::btDefaultMotionState = fall_rigid_body.m_optionalMotionState as *mut _;

            let graphics_world_trans = (*motion_state).m_graphicsWorldTrans;
            let sphere_y = graphics_world_trans.m_origin.m_floats[1];
            assert!(old_y > sphere_y);
            old_y = sphere_y;

            // wow sphere is actually falling down!
            println!("{:?}", sphere_y);

        }
    }
}
