use std::fmt;

#[derive(Debug)]
pub enum Error {
    ConnectionTerminated,
    CommandFailed
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Error::ConnectionTerminated => write!(f, "ConnectionTerminated"),
            Error::CommandFailed => write!(f, "CommandFailed"),
        }
    }
}
