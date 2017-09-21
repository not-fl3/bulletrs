use command::{Command, CommandParam};
use shape::{Shape, ShapeType};
use multibody::{DynamicsInfo, MultiBodyHandle};
use rigidbody::RigidBodyHandle;
use status::Status;
use errors::Error;

use mint::{Vector3, Vector4};

#[derive(Clone)]
pub struct PhysicsClientHandle {
    pub(crate) handle: ::sys::b3PhysicsClientHandle,
}

impl PhysicsClientHandle {
    /// There can only be 1 outstanding command. Check if a command can be send.
    pub fn can_submit_command(&self) -> bool {
        unsafe { ::sys::b3CanSubmitCommand(self.handle) != 0 }
    }

    /// blocking submit command and wait for status
    pub fn submit_client_command_and_wait_status(&self, command: &Command) -> Status {
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
        self.submit_client_command_and_wait_status(&Command::SyncBodyInfo);
    }

    /// Set the gravity acceleration (x,y,z).
    pub fn set_gravity(&self, gravx: f64, gravy: f64, gravz: f64) {
        self.submit_client_command_and_wait_status(
            &Command::PhysicsParam(CommandParam::SetGravity {
                gravx,
                gravy,
                gravz,
            }),
        );
    }

    pub fn set_realtime_simulation(&self, flag: bool) {
        self.submit_client_command_and_wait_status(&Command::PhysicsParam(
            CommandParam::RealTimeSimulation(flag),
        ));
    }

    pub fn create_collision_shape(&self, mesh: ShapeType) -> Result<Shape, Error> {
        let status =
            self.submit_client_command_and_wait_status(&Command::CreateCollisionShape(mesh));

        if status.get_status_type() !=
            ::sys::EnumSharedMemoryServerStatus::CMD_CREATE_COLLISION_SHAPE_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        return Ok(Shape {
            client_handle: self.clone(),
            unique_id: unsafe { ::sys::b3GetStatusCollisionShapeUniqueId(status.handle) },
        });
    }

    pub fn step_simulation(&self) {
        self.submit_client_command_and_wait_status(&Command::StepSimulation);
    }

    pub fn create_multi_body(
        &self,
        shape: Shape,
        mass: f64,
        position: Vector3<f64>,
        orientation: Vector4<f64>,
    ) -> Result<MultiBodyHandle, Error> {
        let status = self.submit_client_command_and_wait_status(&Command::CreateMultiBodyHandle {
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

        return Ok(MultiBodyHandle {
            client_handle: self.clone(),
            unique_id: unsafe { ::sys::b3GetStatusBodyIndex(status.handle) },
        });
    }

    pub fn create_rigid_body(
        &self,
        shape: Shape,
        mass: f64,
        position: Vector3<f64>,
        orientation: Vector4<f64>,
    ) -> Result<RigidBodyHandle, Error> {
        let status = self.submit_client_command_and_wait_status(&Command::CreateRigidBodyHandle {
            shape,
            mass,
            position,
            orientation,
        });

        if status.get_status_type() !=
            ::sys::EnumSharedMemoryServerStatus::CMD_RIGID_BODY_CREATION_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        return Ok(RigidBodyHandle {
            client_handle: self.clone(),
            unique_id: unsafe { ::sys::b3GetStatusBodyIndex(status.handle) },
        });
    }

    /// Get the world position and orientation of the base of the object.
    /// (x,y,z) position vector and (x,y,z,w) quaternion orientation.
    pub fn change_dynamics_info(&self, body: MultiBodyHandle, dynamics_info: DynamicsInfo) {
        self.submit_client_command_and_wait_status(
            &Command::ChangeDynamicsInfo(body, dynamics_info),
        );
    }

    /// Get the positions (x,y,z) and orientation (x,y,z,w) in quaternion
    /// values for the base link of your object
    /// Object is retrieved based on body index, which is the order
    /// the object was loaded into the simulation (0-based)
    pub fn get_base_position_and_orientation(
        &self,
        body: MultiBodyHandle,
    ) -> Result<(Vector3<f64>, Vector4<f64>), Error> {
        let status =
            self.submit_client_command_and_wait_status(
                &Command::GetBasePositionAndOrientation(body),
            );

        if status.get_status_type() !=
            ::sys::EnumSharedMemoryServerStatus::CMD_ACTUAL_STATE_UPDATE_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        #[repr(C)]
        struct ActualState {
            position: Vector3<f64>,
            orientation: Vector4<f64>,
        }
        let mut actual_state = ActualState {
            position: Vector3::from([0.0, 0.0, 0.0]),
            orientation: Vector4::from([0.0, 0.0, 0.0, 0.0]),
        };
        let actual_state_ref = &mut actual_state;
        unsafe {
            ::sys::b3GetStatusActualState(
                status.handle,
                ::std::ptr::null_mut(), /* body_unique_id */
                ::std::ptr::null_mut(), /* num_degree_of_freedom_q */
                ::std::ptr::null_mut(), /* num_degree_of_freedom_u */
                ::std::ptr::null_mut(), /*root_local_inertial_frame*/
                ::std::mem::transmute(&actual_state_ref),
                ::std::ptr::null_mut(), /* actual_state_q_dot */
                ::std::ptr::null_mut(), /* joint_reaction_forces */
            );
        }
        Ok((actual_state_ref.position, actual_state_ref.orientation))
    }
}
