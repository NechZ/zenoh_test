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

apply_patch() {  # repo patch
  local repo=$1 patch=$2
  if git -C "$repo" apply --reverse --check "$patch" >/dev/null 2>&1; then
    echo "[setup] $(basename "$patch"): already applied"
  elif git -C "$repo" apply --check "$patch" >/dev/null 2>&1; then
    git -C "$repo" apply "$patch"
    echo "[setup] $(basename "$patch"): applied"
  else
    echo "[setup] ERROR: $(basename "$patch") neither applies nor is already applied in $repo" >&2
    echo "        (the checkout is probably not at the pinned commit)" >&2
    return 1
  fi
}

clone_pinned "$OUSTER_URL" "$OUSTER_BRANCH" "$OUSTER_COMMIT" "$ROOT/src/ouster-ros"
git -C "$ROOT/src/ouster-ros" submodule update --init --recursive
clone_pinned "$PYLON_URL" "$PYLON_BRANCH" "$PYLON_COMMIT" "$ROOT/src/pylon-ros-camera"

apply_patch "$ROOT/src/ouster-ros"                      "$ROOT/patches/ouster-ros.patch"
apply_patch "$ROOT/src/ouster-ros/ouster-ros/ouster-sdk" "$ROOT/patches/ouster-sdk.patch"
apply_patch "$ROOT/src/pylon-ros-camera"                "$ROOT/patches/pylon-ros-camera.patch"
echo "[setup] sources ready"
