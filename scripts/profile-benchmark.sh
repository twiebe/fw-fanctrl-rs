#!/bin/bash
# Compare two tuned profiles: idle power, single-core, burst and sustained all-core
# throughput, and optionally a real build. The fan is held at a fixed duty so that a
# difference comes from the profile, not from the cooling. The profiles run in
# A B B A order, which cancels slow drift such as a warming room.
#
# usage: sudo [ENV=...] scripts/profile-benchmark.sh PROFILE_A PROFILE_B
#   FAN=60           fixed fan duty in percent
#   IDLE=180         idle seconds (the last two thirds are measured)
#   SINGLE=60        single-core seconds
#   BURST=20         all-core seconds right after a cool-down (inside the PL2 window)
#   HEAT=120         all-core seconds before the sustained run (PL2 budget used up)
#   SUSTAINED=180    sustained all-core seconds
#   COOL=120         cool-down seconds between all-core runs
#   BUILD_CMD=...    timed command, run as the invoking user, e.g.
#                    'cd ~/src/x && cargo clean && cargo build --release'
#   OUT=DIR          result directory (default: a new one under /var/tmp)
#
# Needs stress-ng, tuned-adm and fw-fanctrl. Takes about 1 hour with the defaults (longer with BUILD_CMD).
set -u

FAN=${FAN:-60} IDLE=${IDLE:-180} SINGLE=${SINGLE:-60} BURST=${BURST:-20}
HEAT=${HEAT:-120} SUSTAINED=${SUSTAINED:-180} COOL=${COOL:-120} BUILD_CMD=${BUILD_CMD:-}
RAPL=${RAPL:-/sys/class/powercap/intel-rapl:0}
RAPL_MMIO=${RAPL_MMIO:-/sys/class/powercap/intel-rapl-mmio:0}

if [ $# -ne 2 ]; then
    sed -n '2,/^set -u/p' "$0" | sed -e '/^set -u/d' -e 's/^# \{0,1\}//' >&2
    exit 2
fi
A=$1 B=$2
[ "$(id -u)" -eq 0 ] || { echo "please run as root" >&2; exit 1; }
for tool in stress-ng tuned-adm fw-fanctrl; do
    command -v "$tool" >/dev/null || { echo "$tool not found" >&2; exit 1; }
done
[ -r "$RAPL/energy_uj" ] || { echo "cannot read $RAPL/energy_uj" >&2; exit 1; }
for p in "$A" "$B"; do
    tuned-adm list | grep -q -- "^- $p\( \|-\|$\)" || { echo "unknown profile: $p" >&2; exit 1; }
done
for bat in /sys/class/power_supply/BAT*/status; do
    [ -r "$bat" ] && [ "$(cat "$bat")" = Discharging ] && echo "[Warning] > running on battery" >&2
done

OUT=${OUT:-$(mktemp -d /var/tmp/fw-profile-benchmark.XXXXXX)}
mkdir -p "$OUT"
CSV=$OUT/results.csv
echo "block,profile,metric,value" > "$CSV"
ORIG=$(tuned-adm active | sed 's/.*: //')
LEASE_PID='' FAN_PID=''

cleanup() {
    trap - EXIT INT TERM
    pkill -P $$ stress-ng 2>/dev/null
    [ -n "$LEASE_PID" ] && kill "$LEASE_PID" 2>/dev/null
    [ -n "$FAN_PID" ] && { kill "$FAN_PID" 2>/dev/null; wait "$FAN_PID" 2>/dev/null; }
    fw-fanctrl resume >/dev/null 2>&1
    tuned-adm profile "$ORIG" >/dev/null 2>&1
    echo "fan control and profile '$ORIG' restored" >&2
}
trap cleanup EXIT
trap 'exit 130' INT TERM

energy() { cat "$RAPL/energy_uj"; }

# watts START END SECONDS, handling the counter wrap
watts() {
    local range
    range=$(cat "$RAPL/max_energy_range_uj" 2>/dev/null || echo 0)
    awk -v a="$1" -v b="$2" -v s="$3" -v r="$range" \
        'BEGIN { d = b - a; if (d < 0) d += r; printf "%.2f", d / 1000000 / s }'
}

# effective PL1 in watts: the lower of the MSR and MMIO limits
pl1() {
    cat "$RAPL/constraint_0_power_limit_uw" "$RAPL_MMIO/constraint_0_power_limit_uw" 2>/dev/null |
        awk 'NR == 1 || $1 < m { m = $1 } END { if (NR) printf "%.1f", m / 1000000; else printf "n/a" }'
}

sensors() { fw-fanctrl probe 2>/dev/null | awk '/^Sensor/ { gsub(/[:°C]/, ""); printf "%s=%s ", $2, $3 }'; }

record() { echo "$BLOCK,$PROFILE,$1,$2" >> "$CSV"; printf '  %-22s %s\n' "$1" "$2"; }

# stress NAME WORKERS SECONDS: prints the throughput, records throughput and power
stress() {
    local name=$1 workers=$2 secs=$3 yaml start end ops
    yaml=$OUT/$BLOCK-$name.yaml
    start=$(energy)
    stress-ng --cpu "$workers" --cpu-method matrixprod -t "${secs}s" --metrics-brief \
        --yaml "$yaml" > "$OUT/$BLOCK-$name.log" 2>&1
    end=$(energy)
    ops=$(awk '/bogo-ops-per-second-real-time:/ { print $2; exit }' "$yaml" 2>/dev/null)
    record "$name-ops-per-s" "${ops:-n/a}"
    record "$name-watts" "$(watts "$start" "$end" "$secs")"
}

block() {
    BLOCK=$1 PROFILE=$2
    local start end t0 t1 third=$((IDLE / 3))
    echo "== block $BLOCK: $PROFILE"
    tuned-adm profile "$PROFILE" || exit 1
    sleep "$third"
    start=$(energy)
    sleep "$((IDLE - third))"
    end=$(energy)
    record idle-watts "$(watts "$start" "$end" "$((IDLE - third))")"
    record idle-sensors "$(sensors)"
    stress single 1 "$SINGLE"
    sleep "$COOL"
    stress burst 0 "$BURST"
    sleep "$COOL"
    stress heat 0 "$HEAT"
    stress sustained 0 "$SUSTAINED"
    record sustained-pl1 "$(pl1)"
    record sustained-sensors "$(sensors)"
    if [ -n "$BUILD_CMD" ]; then
        sleep "$COOL"
        t0=$(date +%s.%N)
        if sudo -iu "${SUDO_USER:-root}" sh -c "$BUILD_CMD" > "$OUT/$BLOCK-build.log" 2>&1; then
            t1=$(date +%s.%N)
            record build-seconds "$(awk -v a="$t0" -v b="$t1" 'BEGIN { printf "%.1f", b - a }')"
        else
            record build-seconds failed
        fi
    fi
    sleep "$COOL"
    # The fan hold stops by itself near an EC limit; later numbers would not compare.
    kill -0 "$FAN_PID" 2>/dev/null || { echo "the fan hold ended early, stopping" >&2; exit 1; }
}

# Hand the fan over: the service keeps its pause while the lease is renewed, and
# hands control back by itself if this script dies.
if fw-fanctrl pause --lease 60 >/dev/null 2>&1; then
    ( while sleep 20; do fw-fanctrl pause --lease 60 >/dev/null 2>&1; done ) &
    LEASE_PID=$!
fi
fw-fanctrl test-fan "$FAN" --seconds 14400 >/dev/null 2>&1 &
FAN_PID=$!
sleep 2
kill -0 "$FAN_PID" 2>/dev/null || { echo "could not hold the fan" >&2; exit 1; }
sleep "$COOL"

block 1 "$A"
block 2 "$B"
block 3 "$B"
block 4 "$A"

# Mean per profile, and B relative to A.
SUMMARY=$OUT/summary.txt
awk -F, -v A="$A" -v B="$B" '
    NR > 1 && $4 + 0 == $4 {
        if (!($3 in seen)) { seen[$3] = 1; order[++n] = $3 }
        sum[$2, $3] += $4; cnt[$2, $3]++
    }
    END {
        printf "%-22s %12s %12s %9s\n", "metric", "A", "B", "B vs A"
        for (i = 1; i <= n; i++) {
            m = order[i]
            if (!cnt[A, m] || !cnt[B, m]) continue
            a = sum[A, m] / cnt[A, m]; b = sum[B, m] / cnt[B, m]
            if (a != 0) printf "%-22s %12.2f %12.2f %+8.1f%%\n", m, a, b, (b - a) / a * 100
            else printf "%-22s %12.2f %12.2f %9s\n", m, a, b, "n/a"
        }
        printf "\nA = %s\nB = %s\n", A, B
    }' "$CSV" | tee "$SUMMARY"
echo
echo "results: $OUT"
