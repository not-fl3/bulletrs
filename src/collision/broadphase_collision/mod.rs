use sys;

pub enum BroadphaseInterface {
    AxisSweep3,
    DbvtBroadphase,
    MultiSapBroadphase,
    SimpleBroadphase,
}

pub enum Broadphase {
    DbvtBroadphase(Box<::sys::btDbvtBroadphase>),
}

impl Broadphase {
    pub fn new(interface_type: BroadphaseInterface) -> Self {
        match interface_type {
            BroadphaseInterface::DbvtBroadphase => Broadphase::DbvtBroadphase(
                unsafe { Box::new(sys::btDbvtBroadphase::new(::std::ptr::null_mut())) },
            ),
            _ => unimplemented!(),
        }
    }

    pub fn as_ptr(&self) -> *mut sys::btBroadphaseInterface {
        match self {
            &Broadphase::DbvtBroadphase(ref broadphase) =>
                &**broadphase as *const _ as *mut _
        }
    }

}
impl Drop for Broadphase {
    fn drop(&mut self) {
        match self {
            &mut Broadphase::DbvtBroadphase(ref mut broadphase) => unsafe {
                ::sys::btDbvtBroadphase_btDbvtBroadphase_destructor(&mut **broadphase as *mut _)
            },
        }
    }
}
