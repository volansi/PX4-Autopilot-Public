#!/bin/bash

TEST_DIR="$HOME/.cantst/"
HERE=$(pwd)

cd "$TEST_DIR" || exit

for f in *; do
	# Each directory is a test, go through each directory and extract
	# the necessary files to their correct location
    if [ -d "$f" ]; then
	cd "$TEST_DIR/$f" || exit
	grep -r "counter" > counts.log
	grep -r "timestamp" > times.log
	grep -r "errors" > errors.log
    fi
    cd "$TEST_DIR" || exit
done

cd "$HERE" || exit
