#!/usr/bin/env bash

if [[ -z "$PREFIX" ]]; then
    PREFIX="/usr/local"
   # $var is empty, do what you want
fi


echo "Installing ornis to $PREFIX/bin"

cd "$(dirname "$0")"

if ./compile.sh ; then
    echo "Copying file"
    sudo cp ../../build/ornis/ornis $PREFIX/bin
fi
echo "Installation complete!"
