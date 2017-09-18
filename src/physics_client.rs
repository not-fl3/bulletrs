use command::{Command, CommandParam};
use shape::Shape;
use multibody::{MultiBody, DynamicsInfo};
use status::Status;
use errors::Error;

use std::ops::Drop;
use mint::{Vector3, Vector4};

pub enum ShapeType {
    Sphere { radius: f64 },
    Cylinder,
    Box,
    Capsule,
    Plane { normal: Vector3<f64>, constant: f64 },
    Mesh,
}

pub struct PhysicsClient {
    pub(crate) handle: ::sys::b3PhysicsClientHandle,
}

impl PhysicsClient {
    /// There can only be 1 outstanding command. Check if a command can be send.
    pub fn can_submit_command(&self) -> bool {
        unsafe { ::sys::b3CanSubmitCommand(self.handle) != 0 }
    }

    /// b3DisconnectSharedMemory will disconnect the client from the server and cleanup memory.
    pub fn disconnect_shared_memory(&self) {
        unsafe { ::sys::b3DisconnectSharedMemory(self.handle) }
    }

    /// blocking submit command and wait for status
    pub fn submmit_client_command_and_wait_status(&self, command: &Command) -> Status {
        Status {
            handle: unsafe {
                ::sys::b3SubmitClientCommandAndWaitStatus(
                    self.handle,
                    command.get_handle(self).handle,
                )
            },
        }
    }

    /// Reset the simulation to remove all loaded objects
    pub fn reset_simulation(&self) {
        self.submmit_client_command_and_wait_status(&Command::SyncBodyInfo);
    }

    /// Set the gravity acceleration (x,y,z).
    pub fn set_gravity(&self, gravx: f64, gravy: f64, gravz: f64) {
        self.submmit_client_command_and_wait_status(
            &Command::PhysicsParam(CommandParam::SetGravity {
                gravx,
                gravy,
                gravz,
            }),
        );
    }

    pub fn set_realtime_simulation(&self, flag: bool) {
        self.submmit_client_command_and_wait_status(&Command::PhysicsParam(
            CommandParam::RealTimeSimulation(flag),
        ));
    }

    pub fn create_collision_shape(&self, mesh: ShapeType) -> Result<Shape, Error> {
        let status =
            self.submmit_client_command_and_wait_status(&Command::CreateCollisionShape(mesh));

        if status.get_status_type() !=
            ::sys::EnumSharedMemoryServerStatus::CMD_CREATE_COLLISION_SHAPE_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        return Ok(Shape {
            unique_id: unsafe { ::sys::b3GetStatusCollisionShapeUniqueId(status.handle) },
        });
    }

    pub fn step_simulateion(&self) {
        self.submmit_client_command_and_wait_status(&Command::StepSimulation);
    }

    pub fn create_multi_body(
        &self,
        shape: Shape,
        mass: f64,
        position: Vector3<f64>,
        orientation: Vector4<f64>,
    ) -> Result<MultiBody, Error> {
        let status = self.submmit_client_command_and_wait_status(&Command::CreateMultiBody {
            shape,
            mass,
            position,
            orientation,
        });

        if status.get_status_type() !=
            ::sys::EnumSharedMemoryServerStatus::CMD_CREATE_MULTI_BODY_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        return Ok(MultiBody {
            unique_id: unsafe { ::sys::b3GetStatusBodyIndex(status.handle) },
        });
    }

    pub fn change_dynamics_info(&self, body : MultiBody, dynamics_info : DynamicsInfo) {
        self.submmit_client_command_and_wait_status(&Command::ChangeDynamicsInfo(body, dynamics_info));
    }
}

impl Drop for PhysicsClient {
    fn drop(&mut self) {
        self.disconnect_shared_memory();
    }
}
