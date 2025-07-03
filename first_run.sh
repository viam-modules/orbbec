#!/usr/bin/env bash

set -euo pipefail

OS=$(uname)

if [[ "$OS" == 'Linux' ]]; then
    cd $(dirname $0)

    # Install udev rules
    sudo ./install_udev_rules.sh
fi
