#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"

if [[ "$(uname)" -eq "Linux" ]]; then
    git submodule update --init --recursive
else
    # expect libraries from Daisy Toolchain installation
    git submodule update --init
fi

cd Aurora-SDK
./ci/build_libs.sh

cd -
make
