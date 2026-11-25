#!/bin/bash
# upload-when-robot-connects.sh
#
# Queue a robot code upload. Run this while you're still on normal Wi-Fi, then
# switch your laptop to the Control Hub's Wi-Fi. As soon as the robot shows up,
# this installs the already-built TeamCode app over ADB (the same thing Android
# Studio's Run button does), restarts the Robot Controller app, and pops a Mac
# notification telling you to switch back.
#
# Usage (from the project folder):
#   ./gradlew :TeamCode:assembleDebug            # build first, while online
#   tools/upload-when-robot-connects.sh          # then queue the upload
#   tools/upload-when-robot-connects.sh my.apk   # or upload a specific APK
#   WAIT_MINUTES=30 tools/upload-when-robot-connects.sh
#
# Installing restarts the Robot Controller app, so don't run it mid-OpMode.

set -u

ROBOT_IP="${ROBOT_IP:-192.168.43.1}"
ROBOT_PORT="${ROBOT_PORT:-5555}"
WAIT_MINUTES="${WAIT_MINUTES:-120}"
RC_PACKAGE="com.qualcomm.ftcrobotcontroller"

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
APK_SRC="${1:-$REPO_DIR/TeamCode/build/outputs/apk/debug/TeamCode-debug.apk}"
ADB="${ADB:-${ANDROID_HOME:-$HOME/Library/Android/sdk}/platform-tools/adb}"
TARGET="$ROBOT_IP:$ROBOT_PORT"

log() { echo "[$(date '+%H:%M:%S')] $*"; }
notify() {  # title, message
    log "$1: $2"
    if command -v osascript >/dev/null 2>&1; then
        osascript -e "display notification \"$2\" with title \"$1\" sound name \"Glass\"" >/dev/null 2>&1
    fi
}
adb_state() { "$ADB" -s "$TARGET" get-state 2>/dev/null; }

[ -x "$ADB" ] || { log "adb not found at $ADB"; exit 1; }
[ -f "$APK_SRC" ] || { log "APK not found: $APK_SRC (build it first)"; exit 1; }

# Snapshot the APK now, so a rebuild while we wait can't hand us a half-written file
SNAPSHOT_DIR="$(mktemp -d)"
SNAPSHOT="$SNAPSHOT_DIR/TeamCode-queued.apk"
cp "$APK_SRC" "$SNAPSHOT"
trap 'rm -rf "$SNAPSHOT_DIR"' EXIT
log "Queued $(basename "$APK_SRC") ($(du -h "$SNAPSHOT" | cut -f1 | tr -d ' '), built $(date -r "$APK_SRC" '+%b %d %H:%M'))"
log "Waiting up to $WAIT_MINUTES min for the robot at $TARGET. Switch to the robot Wi-Fi now."

deadline=$(( $(date +%s) + WAIT_MINUTES * 60 ))

wait_for_port() {
    until nc -z -G 2 "$ROBOT_IP" "$ROBOT_PORT" >/dev/null 2>&1; do
        if [ "$(date +%s)" -ge "$deadline" ]; then
            notify "Robot upload gave up" "No robot found in $WAIT_MINUTES min. Nothing was installed."
            exit 2
        fi
        sleep 3
    done
}

# 0 = ADB is connected ("device"), 1 = it would not connect
connect_adb() {
    # Drop any stale entry first (e.g. left by an interrupted install), once
    "$ADB" disconnect "$TARGET" >/dev/null 2>&1
    for try in 1 2 3 4 5 6; do
        out="$("$ADB" connect "$TARGET" 2>&1)"
        # Give the robot up to 20 s to finish the handshake. Don't disconnect while it is still
        # connecting: that cuts off the handshake itself.
        for i in $(seq 1 20); do
            state="$(adb_state)"
            [ "$state" = "device" ] && return 0
            sleep 1
        done
        log "ADB try $try failed: connect said \"$out\", state \"${state:-none}\""
        "$ADB" disconnect "$TARGET" >/dev/null 2>&1
        # Last resort: restart the adb server (this also restarts Android Studio's adb)
        [ "$try" = 5 ] && { "$ADB" kill-server >/dev/null 2>&1; "$ADB" start-server >/dev/null 2>&1; }
    done
    return 1
}

installs=0
while [ "$installs" -lt 6 ]; do
    wait_for_port
    log "Robot found. Connecting ADB..."
    if ! connect_adb; then
        if [ "$(date +%s)" -ge "$deadline" ]; then
            notify "Robot upload failed" "Found the robot, but ADB would not connect."
            exit 3
        fi
        log "ADB would not connect. If this repeats, power-cycle the Control Hub. Trying again..."
        sleep 5
        continue
    fi

    # Safety check: only install onto a device that already runs the FTC Robot Controller
    if ! "$ADB" -s "$TARGET" shell pm path "$RC_PACKAGE" 2>/dev/null | grep -q "package:"; then
        notify "Robot upload stopped" "The device at $TARGET isn't running the FTC Robot Controller. Nothing installed."
        exit 4
    fi

    installs=$((installs + 1))
    log "Installing (attempt $installs)..."
    if "$ADB" -s "$TARGET" install -r -t "$SNAPSHOT"; then
        "$ADB" -s "$TARGET" shell monkey -p "$RC_PACKAGE" -c android.intent.category.LAUNCHER 1 >/dev/null 2>&1
        # Save the robot's hardware configs so port/name problems can be checked from the laptop
        "$ADB" -s "$TARGET" shell 'for f in /sdcard/FIRST/*.xml; do echo "== $f"; cat "$f"; echo; done' \
            > "$(dirname "$0")/robot-config-dump.txt" 2>/dev/null
        # And the settings the tuner OpModes saved (turret limits, measured ticks/degree, clencher,
        # ball counter, shot table, sentry...), so they can be checked from the laptop too
        "$ADB" -s "$TARGET" shell 'for f in /sdcard/FIRST/*.txt /sdcard/FIRST/*.csv; do echo "== $f"; cat "$f"; echo; done' \
            > "$(dirname "$0")/robot-settings-dump.txt" 2>/dev/null
        notify "Robot code uploaded" "Robot Controller is restarting. You can switch back to your normal Wi-Fi."
        exit 0
    fi
    # Usually the laptop's Wi-Fi hopped back to another network mid-transfer
    log "Install did not finish (Wi-Fi dropped?). Waiting for the robot again..."
    "$ADB" disconnect "$TARGET" >/dev/null 2>&1
    sleep 3
done

notify "Robot upload failed" "adb install failed $installs times. Try the Run button in Android Studio."
exit 5
