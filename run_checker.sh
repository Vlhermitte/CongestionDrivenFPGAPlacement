#!/bin/bash

# set working dir to script location
cd "$(dirname "$0")"

# Take args : input file, output file
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <input_file> <output_file>"
    exit 1
fi
INPUT_FILE="$1"
OUTPUT_FILE="$2"


# Check docker deamon is running
if ! docker info > /dev/null 2>&1; then
    echo "Docker daemon is not running. Please start Docker and try again."
    exit 1
fi

docker run --rm --platform linux/amd64 -v "$PWD":/app -w /app ubuntu:latest ./checker "$INPUT_FILE" "$OUTPUT_FILE"
EXIT_CODE=$?
exit $EXIT_CODE