use std::fmt;

#[derive(Debug, PartialEq)]
pub enum Error {
    ConnectionTerminated,
    CommandFailed,
    NoValue,
    BodyDeleted
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Error::ConnectionTerminated => write!(f, "ConnectionTerminated"),
            Error::CommandFailed => write!(f, "Command failed, real error probably in bullet's log"),
            Error::NoValue => write!(f, "No such value"),
            Error::BodyDeleted => write!(f, "Trying to use deleted body"),
        }
    }
}
