#!/usr/bin/env bash
set -euo pipefail

if command -v arm-none-eabi-gdb >/dev/null 2>&1; then
    exec arm-none-eabi-gdb "$@"
fi

for candidate in \
    "$HOME/.local/share/stm32cube/bundles/gnu-gdb-for-stm32"/*/bin/arm-none-eabi-gdb \
    "$HOME/.local/share/stm32cube/bundles/gnu-tools-for-stm32"/*/bin/arm-none-eabi-gdb
do
    if [ -x "$candidate" ]; then
        exec "$candidate" "$@"
    fi
done

echo "arm-none-eabi-gdb not found. Install it or add it to PATH." >&2
exit 127
