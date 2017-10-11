use command::{Command, CommandParam};
use shape::{Shape, ShapeType};
use dynamicsinfo::DynamicsInfo;
use rigidbody::RigidBodyHandle;
use status::Status;
use errors::Error;

use mint::{Point3, Vector3, Vector4};

#[derive(Clone)]
pub struct PhysicsClientHandle {
    pub(crate) handle: ::sys::b3PhysicsClientHandle,
}

#[repr(C)]
pub struct BodyActualState {
    pub position: Vector3<f64>,
    pub orientation: Vector4<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
}

pub struct RayHitInfo {
    pub fraction: f64,
    pub body: Option<RigidBodyHandle>,
    pub position: Point3<f64>,
    pub normal: Point3<f64>,
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

    pub fn set_time_stamp(&self, delta: f64) {
        self.submit_client_command_and_wait_status(
            &Command::PhysicsParam(CommandParam::TimeStamp(delta)),
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

        if status.get_status_type()
            != ::sys::EnumSharedMemoryServerStatus::CMD_CREATE_COLLISION_SHAPE_COMPLETED
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

    pub fn create_rigid_body(
        &self,
        shape: Shape,
        mass: f64,
        position: Vector3<f64>,
        orientation: Vector4<f64>,
    ) -> Result<RigidBodyHandle, Error> {
        let status = self.submit_client_command_and_wait_status(&Command::CreateRigidBody {
            shape,
            mass,
            position,
            orientation,
        });

        if status.get_status_type()
            != ::sys::EnumSharedMemoryServerStatus::CMD_CREATE_MULTI_BODY_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        return Ok(RigidBodyHandle {
            client_handle: self.clone(),
            unique_id: unsafe { ::sys::b3GetStatusBodyIndex(status.handle) },
        });
    }

    pub fn remove_rigid_body(&self, body: RigidBodyHandle) {
        self.submit_client_command_and_wait_status(&Command::RemoveRigidBody(body));
    }

    /// Get the world position and orientation of the base of the object.
    /// (x,y,z) position vector and (x,y,z,w) quaternion orientation.
    pub fn change_dynamics_info(&self, body: RigidBodyHandle, dynamics_info: DynamicsInfo) {
        self.submit_client_command_and_wait_status(
            &Command::ChangeDynamicsInfo(body, dynamics_info),
        );
    }

    pub fn get_body_actual_state(&self, body: RigidBodyHandle) -> Result<BodyActualState, Error> {
        let status =
            self.submit_client_command_and_wait_status(
                &Command::GetBasePositionAndOrientation(body),
            );

        if status.get_status_type()
            != ::sys::EnumSharedMemoryServerStatus::CMD_ACTUAL_STATE_UPDATE_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        let mut q_ref: &mut [f64; 7] = unsafe { ::std::mem::uninitialized() };
        let mut qdot_ref: &mut [f64; 6] = unsafe { ::std::mem::uninitialized() };
        //let actual_state_qdot_ref = &mut actual_state.linear_velocity;
        unsafe {
            ::sys::b3GetStatusActualState(
                status.handle,
                ::std::ptr::null_mut(), /* body_unique_id */
                ::std::ptr::null_mut(), /* num_degree_of_freedom_q */
                ::std::ptr::null_mut(), /* num_degree_of_freedom_u */
                ::std::ptr::null_mut(), /*root_local_inertial_frame*/
                &mut q_ref as *mut _ as *mut _,
                &mut qdot_ref as *mut _ as *mut _, /* actual_state_q_dot */
                ::std::ptr::null_mut(),            /* joint_reaction_forces */
            );
        }

        Ok(BodyActualState {
            position: Vector3::from([q_ref[0], q_ref[1], q_ref[2]]),
            orientation: Vector4::from([q_ref[3], q_ref[4], q_ref[5], q_ref[6]]),
            linear_velocity: Vector3::from([qdot_ref[0], qdot_ref[1], qdot_ref[2]]),
            angular_velocity: Vector3::from([qdot_ref[3], qdot_ref[4], qdot_ref[5]]),
        })
    }

    pub fn set_user_data<T: 'static>(&self, body: RigidBodyHandle, data: Box<T>) {
        let pointer = Box::into_raw(data);

        self.submit_client_command_and_wait_status(
            &Command::SetUserPointer(body, pointer as *mut _),
        );
    }

    pub fn get_user_data<T: 'static>(&self, body: RigidBodyHandle) -> Result<Box<T>, Error> {
        let status = self.submit_client_command_and_wait_status(&Command::GetUserPointer(body));
        if status.get_status_type()
            != ::sys::EnumSharedMemoryServerStatus::CMD_GET_USER_POINTER_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        let mut value: &mut T = unsafe { ::std::mem::uninitialized() };
        let pointer: *mut _ = &mut value as *mut _ as *mut _;
        unsafe {
            ::sys::b3GetUserPointer(status.handle, pointer);
        }

        Ok(unsafe { Box::from_raw((*pointer) as *mut _) })
    }

    /// Cast the world with ray, constructed by start and end points.
    /// Begin and end is bounds of colliding segment.
    pub fn raycast(&self, start: Point3<f64>, end: Point3<f64>) -> Result<Vec<RayHitInfo>, Error> {
        let status = self.submit_client_command_and_wait_status(&Command::Raycast(start, end));
        if status.get_status_type()
            != ::sys::EnumSharedMemoryServerStatus::CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        let mut raycast_info: ::sys::b3RaycastInformation = unsafe { ::std::mem::uninitialized() };
        unsafe {
            ::sys::b3GetRaycastInformation(self.handle, &mut raycast_info as *mut _);
        }

        let raycast_rays_info = unsafe {
            ::std::slice::from_raw_parts(raycast_info.m_rayHits, raycast_info.m_numRayHits as usize)
        };

        // there was only 1 ray, so there should be only 1 output
        let ray_info = raycast_rays_info.get(0).ok_or(Error::CommandFailed)?;

        let hits =
            unsafe { ::std::slice::from_raw_parts(ray_info.hits, ray_info.m_numHits as usize) };

        Ok(
            hits.iter()
                .map(|hit| {
                    RayHitInfo {
                        fraction: hit.m_hitFraction,
                        body: if hit.m_hitObjectUniqueId == -1 {
                            None
                        } else {
                            Some(RigidBodyHandle {
                                client_handle: self.clone(),
                                unique_id: hit.m_hitObjectUniqueId,
                            })
                        },
                        position: Point3::from(hit.m_hitPositionWorld),
                        normal: Point3::from(hit.m_hitNormalWorld),
                    }
                })
                .collect(),
        )
    }
}
