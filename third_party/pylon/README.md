# Basler pylon installers (not in git)

The Docker image installs the pylon SDK (it provides `/opt/pylon`, the pylon libraries the ROS driver and
the pure-Zenoh app link against, and the **camera emulator** that replaces real cameras). Basler's license
does not allow redistribution, so download the packages yourself and put these two files in this directory:

| File | Used for |
|---|---|
| `pylon_25.11.0-deb0_amd64.deb` | the pylon SDK (about 1.1 GB; it is inside Basler's `pylon-25.11.0_linux-x86_64_debs.tar.gz`) |
| `pylon-supplementary-package-for-blaze-1.7.3.73dbe706a_amd64.deb` | blaze support, which the pylon ROS driver build requires |

Get them from Basler's software downloads (pylon Software Suite for Linux, plus the blaze supplementary
package).

The Dockerfile bind-mounts this directory during `docker compose build`, so the files are **not** copied into
an image layer. Newer pylon versions probably work but were not tested: if you use another version, adjust the
file names the Dockerfile expects (`pylon_*.deb`, `pylon-supplementary-package-for-blaze*.deb`).

Files here are ignored by git (see `.gitignore`).
