use physics_client::PhysicsClientHandle;
use command::Command;
use status::EnumSharedMemoryServerStatus;
use errors::Error;

use std::ffi::CString;

pub enum ConnectMethod {
    Gui,
    Direct,
}

pub struct Bullet {
    pub(crate) handle: ::sys::b3PhysicsClientHandle,
}

impl Bullet {
    pub fn connect(method: ConnectMethod) -> Result<Bullet, Error> {
        let client = PhysicsClientHandle {
            handle: match method {
                ConnectMethod::Direct => unsafe { ::sys::b3ConnectPhysicsDirect() },
                ConnectMethod::Gui => unsafe {
                    let mut argv = vec![CString::new("unused").unwrap().into_raw()];
                    ::sys::b3CreateInProcessPhysicsServerAndConnect(1, argv.as_mut_ptr())
                },
            },
        };

        if client.can_submit_command() {
            let status = client.submit_client_command_and_wait_status(&Command::SyncBodyInfo);
            let status_type = status.get_status_type();

            if status_type != EnumSharedMemoryServerStatus::CMD_SYNC_BODY_INFO_COMPLETED {
                return Err(Error::ConnectionTerminated);
            }
            return Ok(Bullet {
                handle : client.handle
            });
        }

        return Err(Error::ConnectionTerminated);
    }

    pub fn physics_client_handle(&self) -> PhysicsClientHandle {
        PhysicsClientHandle {
            handle : self.handle
        }
    }
}

impl Drop for Bullet {
    fn drop(&mut self) {
        unsafe { ::sys::b3DisconnectSharedMemory(self.handle) }
    }
}
