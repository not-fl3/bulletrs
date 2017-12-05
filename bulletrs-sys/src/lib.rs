#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals, dead_code)]
#![cfg_attr(all(target_os = "windows", target_arch = "x86"), feature(abi_thiscall, const_fn))]

#[cfg(all(target_os = "windows", target_arch = "x86"))]
mod bt_bullet_dynamics_common_win_32;

#[cfg(all(target_os = "windows", target_arch = "x86"))]
pub use bt_bullet_dynamics_common_win_32::*;

#[cfg(target_arch = "x86_64")]
mod bt_bullet_dynamics_common;

#[cfg(target_arch = "x86_64")]
pub use bt_bullet_dynamics_common::*;
