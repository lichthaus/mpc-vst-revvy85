#!/bin/sh
# Packages a release for the Open MPC catalogue from a local build, with the catalogue's own tools.
#
#   OPENMPC=/path/to/open-mpc/mpc-vst-plugins MPC_SDK=/path/to/mpc-plugin-builder-sdk release/release.sh 0.4.0
#
# Writes dist/Revvy85-<version>-mpc-aarch64.zip and checks it (catalog_check.py --catalog and
# install_smoke.py). The build output must not contain an EPROM; this refuses to package one.
set -e
cd "$(dirname "$0")/.."
V="$1"; [ -n "$V" ] || { echo "usage: release/release.sh X.Y.Z"; exit 1; }
: "${OPENMPC:?set OPENMPC to a checkout of open-mpc/mpc-vst-plugins}"
grep -q "\"version\": \"$V\"" plugin/Revvy85/plugin.json || { echo "plugin.json is not version $V"; exit 1; }
plugin/Revvy85/make.sh
F="plugin/Revvy85/build/Lichthaus - VST - Revvy85"
if find "$F" -type f \( -iname '*.bin' -o -iname '*.rom' \) | grep -q .; then echo "refusing: a ROM file is in $F"; exit 1; fi
rm -rf dist && mkdir -p dist
python3 -I "$OPENMPC/tools/sdk_release.py" "$F" --version "$V" --id revvy85 --repo lichthaus/mpc-vst-revvy85 --license MIT -o dist -- \
    --user-data Content/roms --user-files release/user-files.json \
    --requires "Your own V1.0 EPROM dump of the unit (rev7_v1.0.bin, 32 KB); not included"
Z="dist/Revvy85-$V-mpc-aarch64.zip"
python3 -I "$OPENMPC/tools/catalog_check.py" "$Z" --catalog
# install_smoke.py runs install.sh, which reads /proc/mounts: Linux (or the MPC) only.
if [ -r /proc/mounts ]; then python3 -I "$OPENMPC/tools/install_smoke.py" "$Z"; else echo "install_smoke.py skipped: no /proc/mounts here (run it on Linux)"; fi
if unzip -l "$Z" | grep -i -E '\.(bin|rom)$'; then echo "a ROM file is in the zip"; exit 1; fi
shasum -a 256 "$Z"
