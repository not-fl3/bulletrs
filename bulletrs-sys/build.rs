extern crate cc;

mod build_linux;
mod build_windows;
mod build_macos;

use std::env;

fn main() {
    if env::var("TARGET").unwrap().contains("windows") {
        build_windows::build_windows();
    }
    else if env::var("TARGET").unwrap().contains("linux") {
        build_linux::build_linux();
    } if env::var("TARGET").unwrap().contains("linux") {
        build_macos::build_macos();
    } else {
        panic!("Unsupported platform!");
    }

}
