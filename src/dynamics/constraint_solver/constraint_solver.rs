use sys;

pub struct ConstraintSolver {
    solver: Box<sys::btSequentialImpulseConstraintSolver>,
}

impl ConstraintSolver {
    pub fn new() -> Self {
        ConstraintSolver {
            solver: Box::new(unsafe { sys::btSequentialImpulseConstraintSolver::new() }),
        }
    }

    pub fn as_ptr(&self) -> *mut sys::btConstraintSolver {
        &*self.solver as *const _ as *mut _
    }
}

impl Drop for ConstraintSolver {
    fn drop(&mut self) {
        unsafe {
            sys::btSequentialImpulseConstraintSolver_btSequentialImpulseConstraintSolver_destructor(&mut *self.solver as *mut _);
        }
    }
}
