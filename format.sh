#!/usr/bin/env bash

set -e

DIRS=(
    "Core/Inc"
    "Core/Src"
    "Core/Tasks"
    "Core/Transports"
    "Core/utils"
    "Drivers/Devices"
)

if ! command -v clang-format >/dev/null 2>&1; then
    echo "Error: clang-format is not installed."
    exit 1
fi

echo "Formatting C source files..."

find "${DIRS[@]}" \
    -type f \
    \( -name "*.c" -o -name "*.h" \) \
    -print0 |
    xargs -0 -r clang-format -i

echo "Done."
