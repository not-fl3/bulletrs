use sys;

pub trait TypedConstraint {
    fn as_ptr(&self) -> *mut sys::btTypedConstraint;
}
