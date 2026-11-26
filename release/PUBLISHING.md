# Publishing Revvy85 (checklist)

Status 2026-11-26: steps 1-4 done. Repo https://github.com/lichthaus/mpc-vst-revvy85, release v0.4.0,
catalogue PR https://github.com/open-mpc/mpc-vst-plugins/pull/1206 (waiting for a maintainer's review).
For a new version: bump plugin.json, run `release/release.sh X.Y.Z`, test on the device, update
`tested.json`, tag `vX.Y.Z` and attach the zip; the catalogue picks it up with no new PR.

1. **Device test of the release zip**: done 2026-11-26 on the Key 37 (MPC OS 3.10.1.12): loads and sounds
   right (owner). Recorded in `tested.json`.
2. **Public repo** `lichthaus/mpc-vst-revvy85`: push `main` (commits as
   `Lichthaus <78316715+lichthaus@users.noreply.github.com>` only), tag `v0.4.0`.
3. **GitHub release** `v0.4.0` with `Revvy85-0.4.0-mpc-aarch64.zip` attached (re-run
   `release/release.sh 0.4.0` first if anything changed; mark it a pre-release to keep it on the
   catalogue's beta channel).
4. **Catalogue PR** to open-mpc/mpc-vst-plugins from a topic branch of a fork, adding
   `catalog/plugins/revvy85.json` = `release/catalog-entry.json`, using their PR template. Paste:
   - `python3 tools/catalog_check.py Revvy85-0.4.0-mpc-aarch64.zip --catalog` output
     (local run 2026-11-25: `OK: revvy85 0.4.0 (0 warnings)`);
   - the zip's SHA-256 (local build 2026-11-25: `a70bfb902cbf9f46ac49e3dbfb13f528c4635ef8d9644df4e9c2dfc5d82f70be`; use the uploaded zip's);
   - `install_smoke.py`: not run (needs Linux `/proc/mounts`).

## PR text draft

> Adds Revvy85, a REV7-style 1985 digital reverb for Gen2: an emulated HD6303R, two YM3804 DSPs and
> a behavioural YM3807 running the user's own V1.0 EPROM dump. No firmware is included or fetched;
> the manifest declares the EPROM in `user_files` (`Content/roms`, 32 KB, `rev7_v1.0.bin`), and the
> plugin passes audio through without it. MIT, built with the Akai MPC Plugin Builder SDK.
> No manufacturer names or logos in the plugin name, skin or screenshot.
