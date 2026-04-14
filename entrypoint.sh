#!/bin/bash
set -e
mkdir -p /workspace
if [ -t 0 ]; then
    echo "No tar input detected. Starting empty workspace"
else
    echo "Extracting workspace from stdin..."
    tar -x -C /workspace -f -
fi

cd /workspace
exec bash