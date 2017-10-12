use std::fmt;

#[derive(Debug, PartialEq)]
pub enum Error {
    ConnectionTerminated,
    CommandFailed,
    NoValue
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Error::ConnectionTerminated => write!(f, "ConnectionTerminated"),
            Error::CommandFailed => write!(f, "CommandFailed"),
            Error::NoValue => write!(f, "No such value"),
        }
    }
}
