use sys;
use {BulletVector3, RigidBodyHandle, Vector3};
use super::TypedConstraint;

pub struct HingeConstraint {
    constraint: Box<sys::btHingeConstraint>,
}

impl HingeConstraint {
    pub fn new<
        T: Into<Vector3<f64>>,
        T1: Into<Vector3<f64>>,
        T2: Into<Vector3<f64>>,
        T3: Into<Vector3<f64>>,
    >(
        rb_a: &RigidBodyHandle,
        rb_b: &RigidBodyHandle,
        pivot_in_a: T,
        pivot_in_b: T1,
        axis_in_a: T2,
        axis_in_b: T3,
        use_reference_frame_a: bool,
    ) -> HingeConstraint {
        let pivot_in_a: BulletVector3 = pivot_in_a.into().into();
        let pivot_in_b: BulletVector3 = pivot_in_b.into().into();
        let axis_in_a: BulletVector3 = axis_in_a.into().into();
        let axis_in_b: BulletVector3 = axis_in_b.into().into();

        HingeConstraint {
            constraint: unsafe {
                Box::new(sys::btHingeConstraint::new(
                    rb_a.ptr,
                    rb_b.ptr,
                    pivot_in_a.0.as_ptr() as * const _,
                    pivot_in_b.0.as_ptr() as * const _,
                    axis_in_a.0.as_ptr() as * const _,
                    axis_in_b.0.as_ptr() as * const _,
                    use_reference_frame_a,
                ))
            },
        }
    }
}

impl TypedConstraint for HingeConstraint {
    fn as_ptr(&self) -> *mut sys::btTypedConstraint {
        &*self.constraint as *const _ as *mut _
    }
}
