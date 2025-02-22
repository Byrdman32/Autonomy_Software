#!/bin/bash

# Define the path to the executable
if [ "$1" = "GitHub-Action" ]; then
  executable_path="/opt/Autonomy_Software/build/Autonomy_Software"
else
  executable_path="/workspaces/Autonomy_Software/build/Autonomy_Software"
fi

# Check if the executable exists
if [ ! -f "$executable_path" ]; then
  echo "Executable not found or hasn't been compiled yet."
  exit 1
fi

# Get the directory of the script
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Attempt to copy the suppressions files from OpenCV install to project directory.
cp /usr/local/share/opencv*/*.supp $script_dir

# Find all .supp files in the same directory as the script
supp_files=$(find "$script_dir" -maxdepth 1 -type f -name "*.supp")

# Construct the Valgrind command
valgrind_cmd="valgrind -s --leak-check=yes --show-leak-kinds=all --track-origins=yes "
if [ "$1" = "GitHub-Action" ]; then
  valgrind_cmd+=" --log-file=/opt/Autonomy_Software/tools/valgrind/valgrind.rpt"
else
  valgrind_cmd+=" --log-file=/workspaces/Autonomy_Software/tools/valgrind/valgrind.rpt"
fi
for supp_file in $supp_files; do
  valgrind_cmd+=" --suppressions=$supp_file"
done
valgrind_cmd+=" $executable_path"

# Run the Valgrind command
timeout --signal=SIGINT 60s $valgrind_cmd

exit_code=$?

if [ $exit_code -eq 124 ] || [ $exit_code -eq 0 ]; then
    exit 0
else
    exit 1
fi