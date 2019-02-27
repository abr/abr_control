#!/usr/bin/env bash
if [[ ! -e .ci/common.sh || ! -e abr_control ]]; then
    echo "Run this script from the root directory of this repository"
    exit 1
fi
source .ci/common.sh

# This script runs the test suite

NAME=$0
COMMAND=$1

if [[ "$COMMAND" == "install" ]]; then
    exe pip install -e .[tests]
elif [[ "$COMMAND" == "script" ]]; then
    exe pytest -v -n 2 --color=yes --durations 20 abr_control
elif [[ -z "$COMMAND" ]]; then
    echo "$NAME requires a command like 'install' or 'script'"
else
    echo "$NAME does not define $COMMAND"
fi

exit "$STATUS"
