#!/bin/sh
# Install fw-fanctrl-rs on an ostree system (e.g. Bazzite) or any systemd distro.
#
# The binary goes to /usr/local/bin (writable on ostree, and ahead of /usr/bin in
# PATH). The units go to /etc/systemd/system, where they take precedence over
# vendor units of the same name, so a preinstalled fw-fanctrl is replaced without
# touching /usr. An existing /etc/fw-fanctrl/config.json is kept.
#
# usage: sudo ./install.sh [path/to/fw-fanctrl]    install / update
#        sudo ./install.sh --uninstall             remove (vendor units become active again)
set -eu

BINDIR=/usr/local/bin
UNITDIR=/etc/systemd/system
CONFDIR=/etc/fw-fanctrl
HERE=$(cd "$(dirname "$0")" && pwd)

if [ "$(id -u)" -ne 0 ]; then
    echo "please run as root" >&2
    exit 1
fi

if [ "${1:-}" = "--uninstall" ]; then
    systemctl disable --now fw-fanctrl-suspend.service fw-fanctrl.service 2>/dev/null || true
    rm -f "$UNITDIR/fw-fanctrl.service" "$UNITDIR/fw-fanctrl-suspend.service" "$BINDIR/fw-fanctrl"
    systemctl daemon-reload
    echo "removed; config in $CONFDIR was kept"
    exit 0
fi

BIN=${1:-$HERE/target/release/fw-fanctrl}
if [ ! -x "$BIN" ]; then
    echo "binary not found: $BIN (build with 'cargo build --release' or pass the path)" >&2
    exit 1
fi

systemctl stop fw-fanctrl-suspend.service fw-fanctrl.service 2>/dev/null || true

install -D -m 0755 "$BIN" "$BINDIR/fw-fanctrl"
for unit in fw-fanctrl.service fw-fanctrl-suspend.service; do
    sed "s|@BINDIR@|$BINDIR|g" "$HERE/systemd/$unit" > "$UNITDIR/$unit"
    chmod 0644 "$UNITDIR/$unit"
done
install -d -m 0755 "$CONFDIR"
install -m 0644 "$HERE/resources/config.schema.json" "$CONFDIR/config.schema.json"
[ -e "$CONFDIR/config.json" ] || install -m 0644 "$HERE/resources/config.json" "$CONFDIR/config.json"

systemctl daemon-reload
systemctl enable --now fw-fanctrl.service
# only enable the sleep hook: starting it now would pause/resume the service once
systemctl enable fw-fanctrl-suspend.service
echo "installed; check with: fw-fanctrl print"
