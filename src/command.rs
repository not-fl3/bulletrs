use physics_client::{PhysicsClient, ShapeType};
use shape::Shape;

use mint::{Vector3, Vector4};

pub enum Command {
    /// If you re-connected to an existing server, or server changed otherwise, sync the body info and user constraints etc.
    SyncBodyInfo,

    PhysicsParam(CommandParam),

    CreateCollisionShape(ShapeType),

    StepSimulation,

    CreateMultiBody {
        shape: Shape,
        mass: f64,
        position: Vector3<f64>,
        orientation: Vector4<f64>,
    },
}

pub enum CommandParam {
    SetGravity { gravx: f64, gravy: f64, gravz: f64 },
    RealTimeSimulation(bool),
}

impl Command {
    pub fn get_handle(&self, client: &PhysicsClient) -> CommandHandle {
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

            &Command::CreateCollisionShape(ref shape) => {
                let command = unsafe { ::sys::b3CreateCollisionShapeCommandInit(client.handle) };
                match shape {
                    &ShapeType::Plane { normal, constant } => {
                        let mut normal: [f64; 3] = normal.into();
                        unsafe {
                            ::sys::b3CreateCollisionShapeAddPlane(
                                command,
                                &mut normal[0] as *mut f64,
                                constant,
                            )
                        }
                    }
                    &ShapeType::Sphere { radius } => unsafe {
                        ::sys::b3CreateCollisionShapeAddSphere(command, radius)
                    },

                    _ => unimplemented!(),
                };

                CommandHandle { handle: command }
            }
            &Command::StepSimulation => {
                let command = unsafe { ::sys::b3InitStepSimulationCommand(client.handle) };
                CommandHandle { handle: command }
            }

            &Command::CreateMultiBody {
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
                CommandHandle { handle : command }
            }
        }
    }
}
pub struct CommandHandle {
    pub(crate) handle: ::sys::b3SharedMemoryCommandHandle,
}
