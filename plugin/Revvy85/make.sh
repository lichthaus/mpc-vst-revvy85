#!/bin/sh
# Builds the plugin end to end: SDK build (writes build/params.json), the PANEL skin (make_skin.py,
# into the git-ignored skins/), SDK build again with the skin, then, when you give it your own EPROM
# dump, stages it into the git-ignored build output and runs the SDK's host_sim against it.
#
#   MPC_SDK=/path/to/mpc-plugin-builder-sdk ./make.sh [path/to/your/rev7_v1.0.bin]
#
# Gen 2 (64-bit) only: plugin.json has no file32, which the SDK validator reports as an error, so
# the build skips it (the Open MPC docs list this as expected for a single-arch release).
# make_skin.py needs Pillow (run through uv here) and the macOS system fonts Helvetica and Menlo.
set -e
cd "$(dirname "$0")"
SDK="${MPC_SDK:-$HOME/mpc-plugin-builder-sdk}"
export MPC_SDK="$SDK"
OUT="build/Lichthaus - VST - Revvy85"
python3 "$SDK/tools/build.py" "$PWD" --target arm64 --no-validate >/dev/null
python3 "$SDK/tools/build.py" "$PWD" --target native >/dev/null
uv run -q --with pillow python3 make_skin.py
python3 "$SDK/tools/build.py" "$PWD" --target arm64 --no-validate | grep -E " errors|error:|warning:" || true
python3 "$SDK/tools/build.py" "$PWD" --target native | grep -E " errors|error:|warning:" || true
[ -n "$1" ] || { echo "built $OUT (no EPROM given: host_sim skipped)"; exit 0; }
for d in "$OUT/Content" build/native/Content; do
    mkdir -p "$d/roms"
    cp "$1" "$d/roms/rev7_v1.0.bin"
done
MPC_PLUGIN_FOLDER="$PWD/$OUT" "$SDK/.sdk-tools/host_sim" build/native/revvy85.dylib test --seconds 6 | grep -E "FAIL|WARN|Result|timing|chunk is" || true
# The EPROM copy stays in the git-ignored build output only; remove it before packaging a release.
rm -f "$OUT/Content/roms/rev7_v1.0.bin" build/native/Content/roms/rev7_v1.0.bin
