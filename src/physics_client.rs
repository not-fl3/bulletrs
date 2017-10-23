use command::{Command, CommandParam};
use shape::{Shape, ShapeType};
use dynamicsinfo::DynamicsInfo;
use rigidbody::{RigidBody, RigidBodyHandle};
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
    /// This is a number between 0.0 and 1.0 where 0.0 is origin and 1.0 is destination.
    /// If the hit fraction is 0.33333, it means position is located 1/3
    /// of the way between ray begin and ray end
    pub fraction: f64,
    pub body: Option<RigidBodyHandle>,
    pub position: Point3<f64>,
    pub normal: Point3<f64>,
}

impl PhysicsClientHandle {
    fn internal_get_user_pointer(&self, body_id: i32) -> Result<*mut RigidBody, Error> {
        let status = self.submit_client_command_and_wait_status(&Command::GetUserPointer(body_id));
        if status.get_status_type()
            != ::sys::EnumSharedMemoryServerStatus::CMD_GET_USER_POINTER_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        let mut value: &mut RigidBody = unsafe { ::std::mem::uninitialized() };
        let pointer: *mut _ = &mut value as *mut _ as *mut _;
        unsafe {
            ::sys::b3GetUserPointer(status.handle, pointer);
        }

        if unsafe { (*pointer).is_null() } {
            return Err(Error::NoValue);
        } else {
            let pointer = unsafe { *pointer as *mut _ };
            Ok(pointer)
        }
    }

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

    pub fn set_time_step(&self, delta: f64) {
        self.submit_client_command_and_wait_status(
            &Command::PhysicsParam(CommandParam::TimeStep(delta)),
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

        let unique_body_id = unsafe { ::sys::b3GetStatusBodyIndex(status.handle) };

        let body = Box::new(RigidBody::new());
        let pointer = Box::into_raw(body);

        self.submit_client_command_and_wait_status(
            &Command::SetUserPointer(unique_body_id, pointer as *mut _),
        );

        return Ok(RigidBodyHandle {
            rigid_body: pointer as *mut _,
            client_handle: self.clone(),
            unique_id: unique_body_id,
        });
    }

    pub fn remove_rigid_body(&self, body: &RigidBodyHandle) {
        let rigid_body = unsafe { body.rigid_body.as_mut().unwrap() };
        rigid_body.deleted = true;
        self.submit_client_command_and_wait_status(&Command::RemoveRigidBody(body.clone()));
    }

    /// Get the world position and orientation of the base of the object.
    /// (x,y,z) position vector and (x,y,z,w) quaternion orientation.
    pub fn change_dynamics_info(&self, body: RigidBodyHandle, dynamics_info: DynamicsInfo) {
        self.submit_client_command_and_wait_status(
            &Command::ChangeDynamicsInfo(body, dynamics_info),
        );
    }

    pub fn get_body_actual_state(&self, body: RigidBodyHandle) -> Result<BodyActualState, Error> {
        let status = self.submit_client_command_and_wait_status(
            &Command::GetBasePositionAndOrientation(body),
        );

        if status.get_status_type()
            != ::sys::EnumSharedMemoryServerStatus::CMD_ACTUAL_STATE_UPDATE_COMPLETED
        {
            return Err(Error::CommandFailed);
        }

        let q_ref: &mut [f64; 7] = unsafe { ::std::mem::uninitialized() };
        let qdot_ref: &mut [f64; 6] = unsafe { ::std::mem::uninitialized() };
        //let actual_state_qdot_ref = &mut actual_state.linear_velocity;
        unsafe {
            ::sys::b3GetStatusActualState(
                status.handle,
                ::std::ptr::null_mut(), /* body_unique_id */
                ::std::ptr::null_mut(), /* num_degree_of_freedom_q */
                ::std::ptr::null_mut(), /* num_degree_of_freedom_u */
                ::std::ptr::null_mut(), /*root_local_inertial_frame*/
                &q_ref as *const _ as *mut _,
                &qdot_ref as *const _ as *mut _, /* actual_state_q_dot */
                ::std::ptr::null_mut(),          /* joint_reaction_forces */
            );
        }

        Ok(BodyActualState {
            position: Vector3::from([q_ref[0], q_ref[1], q_ref[2]]),
            orientation: Vector4::from([q_ref[3], q_ref[4], q_ref[5], q_ref[6]]),
            linear_velocity: Vector3::from([qdot_ref[0], qdot_ref[1], qdot_ref[2]]),
            angular_velocity: Vector3::from([qdot_ref[3], qdot_ref[4], qdot_ref[5]]),
        })
    }

    /// Cast the world with ray, constructed by start and end points.
    /// Begin and end is bounds of colliding segment.
    /// Results will be in random order.
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

        let ray_info = unsafe { *raycast_info.m_rayHits };
        let hits = ray_info.hits.iter().take(ray_info.m_numHits as usize);
        let hits = hits.map(|hit| {
            let rigid_body = self.internal_get_user_pointer(hit.m_hitObjectUniqueId);
            RayHitInfo {
                fraction: hit.m_hitFraction,
                body: if hit.m_hitObjectUniqueId == -1 || rigid_body.is_err() {
                    None
                } else {
                    Some(RigidBodyHandle {
                        rigid_body: rigid_body.unwrap(),
                        client_handle: PhysicsClientHandle {
                            handle: self.handle,
                        },
                        unique_id: hit.m_hitObjectUniqueId,
                    })
                },
                position: Point3::from(hit.m_hitPositionWorld),
                normal: Point3::from(hit.m_hitNormalWorld),
            }
        }).collect();

        return Ok(hits);
    }

    /// Add debug line for GUI connection type
    /// Will do nothing in Direct or any other modes
    pub fn add_user_debug_line(&self, from: Point3<f64>, to: Point3<f64>, color: Vector4<f64>) {
        self.submit_client_command_and_wait_status(&Command::AddUserDebugLine {
            from,
            to,
            color,
            line_width: 1.0,
            life_time: 666.0,
        });
    }
}
