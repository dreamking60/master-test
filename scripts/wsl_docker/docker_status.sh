#!/usr/bin/env bash
set -euo pipefail

CONTAINERS=("$@")

echo "=== docker ps ==="
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
echo

echo "=== docker stats ==="
if [[ "${#CONTAINERS[@]}" -gt 0 ]]; then
  docker stats --no-stream "${CONTAINERS[@]}" 2>&1 || true
else
  docker stats --no-stream 2>&1 || true
fi
