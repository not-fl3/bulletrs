use physics_client::PhysicsClientHandle;
use shape::{Shape, ShapeType};
use dynamicsinfo::DynamicsInfo;
use rigidbody::RigidBodyHandle;
use mint::{Vector3, Vector4};

pub enum Command {
    /// If you re-connected to an existing server, or server changed otherwise, sync the body info and user constraints etc.
    SyncBodyInfo,

    PhysicsParam(CommandParam),

    CreateCollisionShape(ShapeType),

    StepSimulation,

    CreateRigidBody {
        shape: Shape,
        mass: f64,
        position: Vector3<f64>,
        orientation: Vector4<f64>,
    },

    RemoveRigidBody(RigidBodyHandle),

    ChangeDynamicsInfo(RigidBodyHandle, DynamicsInfo),

    GetBasePositionAndOrientation(RigidBodyHandle),

    SetAngularFactor(RigidBodyHandle, Vector3<f64>),

    ApplyCentralImpulse(RigidBodyHandle, Vector3<f64>),

    SetUserPointer(RigidBodyHandle, *mut ::std::os::raw::c_void),

    GetUserPointer(RigidBodyHandle),

    SetBodyGravity(RigidBodyHandle, Vector3<f64>),
}

pub enum CommandParam {
    SetGravity { gravx: f64, gravy: f64, gravz: f64 },
    TimeStamp(f64),
    RealTimeSimulation(bool),
}

impl Command {
    pub fn get_handle(&self, client: &PhysicsClientHandle) -> CommandHandle {
        match self {
            &Command::SyncBodyInfo => CommandHandle {
                handle: unsafe { ::sys::b3InitSyncBodyInfoCommand(client.handle) },
            },
            &Command::PhysicsParam(CommandParam::SetGravity {
                gravx,
                gravy,
                gravz,
            }) => {
                let command = unsafe { ::sys::b3InitPhysicsParamCommand(client.handle) };
                unsafe { ::sys::b3PhysicsParamSetGravity(command, gravx, gravy, gravz) };
                CommandHandle { handle: command }
            }
            &Command::PhysicsParam(CommandParam::RealTimeSimulation(flag)) => {
                let command = unsafe { ::sys::b3InitPhysicsParamCommand(client.handle) };
                unsafe {
                    ::sys::b3PhysicsParamSetRealTimeSimulation(command, if flag { 1 } else { 0 })
                };
                CommandHandle { handle: command }
            }
            &Command::PhysicsParam(CommandParam::TimeStamp(delta)) => {
                let command = unsafe { ::sys::b3InitPhysicsParamCommand(client.handle) };
                unsafe { ::sys::b3PhysicsParamSetTimeStep(command, delta) };
                CommandHandle { handle: command }
            }
            &Command::CreateCollisionShape(ref body) => {
                let command = unsafe { ::sys::b3CreateCollisionShapeCommandInit(client.handle) };
                body.create_shape(command);
                CommandHandle { handle: command }
            }

            &Command::StepSimulation => {
                let command = unsafe { ::sys::b3InitStepSimulationCommand(client.handle) };
                CommandHandle { handle: command }
            }

            &Command::CreateRigidBody {
                ref shape,
                mass,
                position,
                orientation,
            } => {
                let command = unsafe { ::sys::b3CreateMultiBodyCommandInit(client.handle) };

                let mut position: [f64; 3] = position.into();
                let mut orientation: [f64; 4] = orientation.into();
                let mut base_inertial_frame_position = vec![0.0, 0.0, 0.0];
                let mut base_inertial_frame_orientation = vec![0.0, 0.0, 0.0, 1.0];
                unsafe {
                    ::sys::b3CreateMultiBodyBase(
                        command,
                        mass,
                        shape.unique_id,
                        -1,
                        &mut position[0] as *mut _,
                        &mut orientation[0] as *mut _,
                        &mut base_inertial_frame_position[0] as *mut _,
                        &mut base_inertial_frame_orientation[0] as *mut _,
                    );
                }

                // with USE_MAXIMAL_COORDINATES physcs clients creates btRigidBody instead of btMultiBody
                unsafe {
                    ::sys::b3CreateMultiBodyUseMaximalCoordinates(command);
                }
                CommandHandle { handle: command }
            }

            &Command::RemoveRigidBody(ref shape) => {
                let command =
                    unsafe { ::sys::b3InitRemoveBodyCommand(client.handle, shape.unique_id) };
                CommandHandle { handle: command }
            }

            &Command::ChangeDynamicsInfo(ref body, ref dynamics_info) => {
                let command = unsafe { ::sys::b3InitChangeDynamicsInfo(client.handle) };
                if let Some(mass) = dynamics_info.mass {
                    unsafe {
                        ::sys::b3ChangeDynamicsInfoSetMass(command, body.unique_id, -1, mass)
                    };
                }
                if let Some(lateral_friction) = dynamics_info.lateral_friction {
                    unsafe {
                        ::sys::b3ChangeDynamicsInfoSetLateralFriction(
                            command,
                            body.unique_id,
                            -1,
                            lateral_friction,
                        )
                    };
                }
                if let Some(spinning_friction) = dynamics_info.spinning_friction {
                    unsafe {
                        ::sys::b3ChangeDynamicsInfoSetLateralFriction(
                            command,
                            body.unique_id,
                            -1,
                            spinning_friction,
                        )
                    };
                }
                if let Some(rolling_friction) = dynamics_info.rolling_friction {
                    unsafe {
                        ::sys::b3ChangeDynamicsInfoSetRollingFriction(
                            command,
                            body.unique_id,
                            -1,
                            rolling_friction,
                        )
                    };
                }
                if let Some(linear_damping) = dynamics_info.linear_damping {
                    unsafe {
                        ::sys::b3ChangeDynamicsInfoSetLinearDamping(
                            command,
                            body.unique_id,
                            linear_damping,
                        )
                    };
                }
                if let Some(angular_damping) = dynamics_info.angular_damping {
                    unsafe {
                        ::sys::b3ChangeDynamicsInfoSetAngularDamping(
                            command,
                            body.unique_id,
                            angular_damping,
                        )
                    };
                }
                if let Some(restitution) = dynamics_info.restitution {
                    unsafe {
                        ::sys::b3ChangeDynamicsInfoSetRestitution(
                            command,
                            body.unique_id,
                            -1,
                            restitution,
                        )
                    };
                }
                if let Some((contact_stiffness, contact_damping)) =
                    dynamics_info.contact_stiffness_and_damping
                {
                    unsafe {
                        ::sys::b3ChangeDynamicsInfoSetContactStiffnessAndDamping(
                            command,
                            body.unique_id,
                            -1,
                            contact_stiffness,
                            contact_damping,
                        )
                    };
                }
                if let Some(friction_anchor) = dynamics_info.friction_anchor {
                    unsafe {
                        ::sys::b3ChangeDynamicsInfoSetFrictionAnchor(
                            command,
                            body.unique_id,
                            -1,
                            friction_anchor,
                        )
                    };
                }
                CommandHandle { handle: command }
            }

            &Command::GetBasePositionAndOrientation(ref body) => {
                let command = unsafe {
                    ::sys::b3RequestActualStateCommandInit(client.handle, body.unique_id)
                };

                CommandHandle { handle: command }
            }

            &Command::SetAngularFactor(ref body, factor) => {
                let mut factor: [f64; 3] = factor.into();
                let command = unsafe {
                    ::sys::b3InitSetAngularFactorCommand(
                        client.handle,
                        body.unique_id,
                        factor.as_mut_ptr(),
                    )
                };

                CommandHandle { handle: command }
            }

            &Command::ApplyCentralImpulse(ref body, impulse) => {
                let mut impulse: [f64; 3] = impulse.into();
                let command = unsafe {
                    ::sys::b3InitApplyCentralImpulseCommand(
                        client.handle,
                        body.unique_id,
                        impulse.as_mut_ptr(),
                    )
                };

                CommandHandle { handle: command }
            }

            &Command::SetUserPointer(ref body, ref data) => {
                let command = unsafe {
                    ::sys::b3InitSetUserPointerCommand(
                        client.handle,
                        body.unique_id,
                        *data as *mut _,
                    )
                };
                CommandHandle { handle: command }
            }

            &Command::GetUserPointer(ref body) => {
                let command =
                    unsafe { ::sys::b3InitGetUserPointerCommand(client.handle, body.unique_id) };
                CommandHandle { handle: command }
            }

            &Command::SetBodyGravity(ref body, gravity) => {
                let mut gravity: [f64; 3] = gravity.into();

                let command =
                    unsafe { ::sys::b3InitSetBodyGravityCommand(client.handle, body.unique_id, gravity.as_mut_ptr()) };
                CommandHandle { handle: command }
            }
        }
    }
}
pub struct CommandHandle {
    pub(crate) handle: ::sys::b3SharedMemoryCommandHandle,
}
