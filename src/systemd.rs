//! Minimal sd_notify(3) without libsystemd.

use std::os::linux::net::SocketAddrExt;
use std::os::unix::net::{SocketAddr, UnixDatagram};

pub fn notify(msg: &str) {
    let Ok(path) = std::env::var("NOTIFY_SOCKET") else {
        return;
    };
    let addr = match path.strip_prefix('@') {
        Some(name) => SocketAddr::from_abstract_name(name.as_bytes()),
        None => SocketAddr::from_pathname(&path),
    };
    if let (Ok(addr), Ok(sock)) = (addr, UnixDatagram::unbound()) {
        let _ = sock.send_to_addr(msg.as_bytes(), &addr);
    }
}
