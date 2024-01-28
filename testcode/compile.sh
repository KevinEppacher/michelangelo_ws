#!/bin/bash

# Check if a source file is provided as an argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <source_file>"
    exit 1
fi

source_file="$1"

# Compile the C++ code
#g++ -o my_program "$source_file"
g++ -std=c++11 -pthread -o my_program "$source_file"
#g++ -std=c++11 -cthread -o my_program $1


if [ $? -eq 0 ]; then
    echo "Running the file"
    ./my_program
fi
