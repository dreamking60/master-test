#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SUDOERS_FILE="/etc/sudoers.d/master-test-codex"
TARGET_USER="${SUDO_USER:-$USER}"

if [[ "${1:-}" == "--remove" ]]; then
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "Run with sudo: sudo $0 --remove" >&2
    exit 1
  fi
  rm -f "$SUDOERS_FILE"
  visudo -c
  echo "Removed $SUDOERS_FILE"
  exit 0
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run with sudo: sudo $0" >&2
  exit 1
fi

TMP_FILE="$(mktemp)"
cat > "$TMP_FILE" <<EOF
Cmnd_Alias MASTER_TEST_SCRIPTS = \\
  $PROJECT_ROOT/scripts/wsl_docker/init_sros2_docker.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/up.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/down.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/secure_up.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/start_controller_stack.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/secure_start_controller_stack.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/run_controller.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/run_attacker.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/exec_attacker.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/exec_controller.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/docker_status.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/mitm_up.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/mitm_down.sh, \\
  $PROJECT_ROOT/scripts/wsl_docker/mitm_exec.sh

$TARGET_USER ALL=(root) NOPASSWD: SETENV: MASTER_TEST_SCRIPTS
EOF

visudo -cf "$TMP_FILE"
install -m 0440 "$TMP_FILE" "$SUDOERS_FILE"
rm -f "$TMP_FILE"
visudo -c

echo "Installed $SUDOERS_FILE for user: $TARGET_USER"
echo
echo "Verify with:"
echo "  sudo -n $PROJECT_ROOT/scripts/wsl_docker/down.sh"
echo "  sudo -n $PROJECT_ROOT/scripts/wsl_docker/docker_status.sh tb3-controller tb3-attacker"
