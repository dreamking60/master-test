#!/usr/bin/env bash

resolve_compose_cmd() {
  if docker compose version >/dev/null 2>&1; then
    echo "docker compose"
    return 0
  fi

  if command -v docker-compose >/dev/null 2>&1; then
    echo "docker-compose"
    return 0
  fi

  echo "ERROR: Docker Compose not found. Install docker-compose-v2 or docker-compose." >&2
  return 1
}

