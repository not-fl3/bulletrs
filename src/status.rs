pub use ::sys::EnumSharedMemoryServerStatus;

use std::mem;

pub struct Status {
    pub(crate) handle : ::sys::b3SharedMemoryStatusHandle
}

impl Status {
    pub fn get_status_type(&self) -> EnumSharedMemoryServerStatus {
        unsafe { mem::transmute(::sys::b3GetStatusType(self.handle)) }
    }
}
