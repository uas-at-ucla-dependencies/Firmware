#!/bin/bash

# Exit if any errors are encountered.
set -e

cd "$(dirname "$0")"

python3 -m venv px4_venv
source px4_venv/bin/activate
pip install -r ./Tools/setup/requirements.txt

make clean
