#!/usr/bin/env bash

set -euo pipefail

cd $(dirname $0)

# Install udev rules
sudo ./install_udev_rules.sh
