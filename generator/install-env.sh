#!/bin/bash

set -xe

if [ ! -d "$1" ]; then
    echo "$0 /path/to/cplex/python/package"
    exit -1
fi

python3 -m venv env
. env/bin/activate
pip install --upgrade pip
pip install shapely
pip install scipy
pip install networkx
pip install pycairo
cp -r "$1" env/lib/python*/site-packages/
