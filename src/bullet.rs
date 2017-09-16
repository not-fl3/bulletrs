use physics_client::PhysicsClient;
use command::Command;
use status::EnumSharedMemoryServerStatus;
use errors::Error;

use std::ffi::CString;

pub enum ConnectMethod {
    Gui,
    Direct,
}

pub struct Bullet;

impl Bullet {
    pub fn connect(&self, method: ConnectMethod) -> Result<PhysicsClient, Error> {
        let client = PhysicsClient {
            handle: match method {
                ConnectMethod::Direct => unsafe { ::sys::b3ConnectPhysicsDirect() },
                ConnectMethod::Gui => unsafe {
                    let mut argv = vec![CString::new("unused").unwrap().into_raw()];
                    ::sys::b3CreateInProcessPhysicsServerAndConnect(1, argv.as_mut_ptr())
                },
            },
        };

        if client.can_submit_command() {
            let status = client.submmit_client_command_and_wait_status(&Command::SyncBodyInfo);
            let status_type = status.get_status_type();

            if status_type != EnumSharedMemoryServerStatus::CMD_SYNC_BODY_INFO_COMPLETED {
                return Err(Error::ConnectionTerminated);
            }
            return Ok(client);
        }

        return Err(Error::ConnectionTerminated);
    }
}
