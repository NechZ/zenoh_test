#!/bin/bash
# Fetch the upstream ROS drivers at the pinned commits and apply the local patches from patches/.
# Safe to re-run: existing clones are kept, patches that are already applied are skipped.
#
# The drivers are your team's GitLab repos (ssh), so run this where your ssh key is available
# (host or container with the key forwarded). See patches/README.md for what each patch changes.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

OUSTER_URL=ssh://git@gitlab.curemannheim.de:7999/driverless/ouster-ros.git
OUSTER_BRANCH=feature/unique-pointer-transport
OUSTER_COMMIT=794885374231e5fc3be9028513d3bdf475d450c6
OUSTER_SDK_COMMIT=048319844567afaf4fe411bf0dead2859e6388e5     # submodule ouster-ros/ouster-sdk

PYLON_URL=ssh://git@gitlab.curemannheim.de:7999/driverless/pylon-ros-camera.git
PYLON_BRANCH=feature/unique-pointer-transport
PYLON_COMMIT=39581c05a08d9918f4ce171cf664ade885d03351

clone_pinned() {  # url branch commit dir
  if [ -d "$4/.git" ]; then
    echo "[setup] $4 exists, keeping it"
  else
    echo "[setup] cloning $1 ($2)"
    git clone --branch "$2" "$1" "$4"
    git -C "$4" checkout -q "$3"
  fi
}

# Each repo gets an ORDERED list of patches that build on one another (base changes, then optional latency
# tracing, then fixes). Later patches overlap earlier ones, so once a later patch is applied the earlier ones no
# longer reverse-apply cleanly. State detection therefore finds the LAST patch that reverse-applies (that patch and
# everything before it are in the tree) and applies the ones after it, in order.
apply_patches() {  # repo patch1 [patch2 ...]
  local repo=$1; shift
  local patches=("$@") n=$# last=-1 i
  for ((i = n - 1; i >= 0; i--)); do
    if git -C "$repo" apply --reverse --check "${patches[i]}" >/dev/null 2>&1; then last=$i; break; fi
  done
  for ((i = 0; i <= last; i++)); do echo "[setup] $(basename "${patches[i]}"): already applied"; done
  for ((i = last + 1; i < n; i++)); do
    if git -C "$repo" apply --check "${patches[i]}" >/dev/null 2>&1; then
      git -C "$repo" apply "${patches[i]}"
      echo "[setup] $(basename "${patches[i]}"): applied"
    else
      echo "[setup] ERROR: $(basename "${patches[i]}") does not apply in $repo" >&2
      echo "        (the checkout is probably not at the pinned commit, or an earlier patch is missing)" >&2
      return 1
    fi
  done
}

clone_pinned "$OUSTER_URL" "$OUSTER_BRANCH" "$OUSTER_COMMIT" "$ROOT/src/ouster-ros"
git -C "$ROOT/src/ouster-ros" submodule update --init --recursive
clone_pinned "$PYLON_URL" "$PYLON_BRANCH" "$PYLON_COMMIT" "$ROOT/src/pylon-ros-camera"

P="$ROOT/patches"
apply_patches "$ROOT/src/ouster-ros" "$P/ouster-ros.patch" "$P/latency-trace-ouster-ros.patch" "$P/ouster-ros-stamp-fix.patch"
apply_patches "$ROOT/src/ouster-ros/ouster-ros/ouster-sdk" "$P/ouster-sdk.patch"
apply_patches "$ROOT/src/pylon-ros-camera" "$P/pylon-ros-camera.patch" "$P/latency-trace-pylon-ros-camera.patch"
echo "[setup] sources ready"
