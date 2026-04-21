#!/usr/bin/env bash
set -euo pipefail

curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"
