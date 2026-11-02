//! SIGTERM/SIGINT → stop, SIGHUP → reload configuration.

use std::sync::atomic::{AtomicI32, Ordering};

static PENDING: AtomicI32 = AtomicI32::new(0);

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Signal {
    Stop,
    Reload,
}

extern "C" fn on_signal(sig: libc::c_int) {
    // Only an atomic store: async-signal-safe. A stop request is never downgraded.
    if PENDING.load(Ordering::SeqCst) != libc::SIGTERM {
        PENDING.store(sig, Ordering::SeqCst);
    }
}

pub fn install() {
    for sig in [libc::SIGTERM, libc::SIGINT, libc::SIGHUP] {
        // SAFETY: installs a handler that only touches an atomic.
        unsafe {
            libc::signal(sig, on_signal as *const () as libc::sighandler_t);
        }
    }
}

pub fn take() -> Option<Signal> {
    match PENDING.swap(0, Ordering::SeqCst) {
        0 => None,
        libc::SIGHUP => Some(Signal::Reload),
        _ => Some(Signal::Stop),
    }
}
