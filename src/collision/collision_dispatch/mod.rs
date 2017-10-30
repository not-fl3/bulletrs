use sys;

pub enum CollisionConfiguration {
    Default(
        Box<sys::btDefaultCollisionConstructionInfo>,
        Box<sys::btDefaultCollisionConfiguration>,
    ),
}

impl CollisionConfiguration {
    pub fn new_default() -> Self {
        let info = Box::new(unsafe { sys::btDefaultCollisionConstructionInfo::new() });
        let collision_configuration = Box::new(unsafe {
            sys::btDefaultCollisionConfiguration::new(&*info as *const _)
        });
        CollisionConfiguration::Default(info, collision_configuration)
    }

    pub(crate) fn as_ptr(&self) -> *mut sys::btCollisionConfiguration {
        match self {
            &CollisionConfiguration::Default(_, ref configuration) => {
                &**configuration as *const _ as *mut _
            }
        }
    }
}
pub struct CollisionDispatcher {
    dispatcher: Box<sys::btCollisionDispatcher>,
}

impl CollisionDispatcher {
    pub fn new(configuration: &CollisionConfiguration) -> Self {
        let dispatcher = Box::new(unsafe {
            sys::btCollisionDispatcher::new(configuration.as_ptr())
        });
        CollisionDispatcher { dispatcher }
    }

    pub fn as_ptr(&self) -> *mut sys::btDispatcher {
        &*self.dispatcher as *const _ as *mut _
    }
}

impl Drop for CollisionDispatcher {
    fn drop(&mut self) {
        unsafe {
            ::sys::btCollisionDispatcher_btCollisionDispatcher_destructor(
                &mut *self.dispatcher as *mut _,
            );
        }
    }
}
