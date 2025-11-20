#!/bin/bash

TARGET_DIRS=(
    "/home/zwg/fastlivo/fast_ws_sim/src/FAST-LIVO2/Log/Colmap/images"
    "/home/zwg/fastlivo/fast_ws_sim/src/FAST-LIVO2/Log/Colmap/sparse/0"
)

for dir in "${TARGET_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        rm -rf "$dir"
        echo "Removed: $dir"
    else
        echo "Not found: $dir"
    fi
done

for dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo "Created: $dir"
    else
        echo "Exists: $dir"
    fi
done

