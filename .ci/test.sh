#!/usr/bin/env bash
if [[ ! -e .ci/common.sh || ! -e abr_control ]]; then
    echo "Run this script from the root directory of this repository"
    exit 1
fi
source .ci/common.sh

# This script runs the test suite and collects coverage information

NAME=$0
COMMAND=$1

if [[ "$COMMAND" == "install" ]]; then
    exe pip install numpy
    exe pip install -e ".[tests]"
elif [[ "$COMMAND" == "script" ]]; then
    exe pytest -v -n 2 --color=yes --durations 20 --cov=abr_control abr_control --ignore-glob="*/test_mujoco_config.py"
elif [[ "$COMMAND" == "after_script" ]]; then
    exe eval "bash <(curl -s https://codecov.io/bash)"
elif [[ -z "$COMMAND" ]]; then
    echo "$NAME requires a command like 'install' or 'script'"
else
    echo "$NAME does not define $COMMAND"
fi

exit "$STATUS"
