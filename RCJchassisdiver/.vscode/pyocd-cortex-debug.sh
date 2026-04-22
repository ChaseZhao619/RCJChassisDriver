#!/usr/bin/env bash
set -o pipefail

# Cortex-Debug 1.12.1 waits for "GDB server started ... port", while
# pyOCD 0.44 prints "GDB server listening ... port". Translate that line.
/home/chase/.local/bin/pyocd "$@" 2>&1 \
    | sed -u -E 's/GDB server listening on port/GDB server started on port/'
