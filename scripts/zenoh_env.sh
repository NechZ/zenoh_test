#!/bin/bash
# Source this file to switch Zenoh shared memory (SHM) on or off for ROS 2 (rmw_zenoh) processes.
#
#   source scripts/zenoh_env.sh shm     # SHM on  (512 MB pool, every message >= 512 B goes through SHM)
#   source scripts/zenoh_env.sh net     # SHM off (rmw_zenoh's default in Jazzy)
#
# SHM must be enabled the same way in every process that should use it (publisher AND readers),
# so source this in each shell before starting them. SHM_POOL_MB overrides the pool size.
zenoh_shm() {
  case "$1" in
    shm|on)
      local pool=$(( ${SHM_POOL_MB:-512} * 1024 * 1024 ))
      export ZENOH_CONFIG_OVERRIDE="transport/shared_memory/enabled=true;transport/shared_memory/mode=\"init\";transport/shared_memory/transport_optimization/enabled=true;transport/shared_memory/transport_optimization/pool_size=${pool};transport/shared_memory/transport_optimization/message_size_threshold=512"
      ;;
    net|off)
      unset ZENOH_CONFIG_OVERRIDE
      ;;
    *)
      echo "usage: zenoh_shm shm|net" >&2
      return 1
      ;;
  esac
}
# When sourced with a valid mode, apply it right away. (A script that sources this file without arguments
# passes its OWN arguments on to it, so anything other than a valid mode is ignored here.)
case "${1:-}" in
  shm|on|net|off) zenoh_shm "$1" ;;
esac
true
