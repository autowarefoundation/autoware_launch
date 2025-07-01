#!/usr/bin/env bash

FILE_REFERENCE="$1"
FILE_TARGET="$2"

# extract Displays block
extract_block() {
    awk '
    $0 ~ /^  Displays:/ { in_block=1; print; next }
    in_block && $0 ~ /^   / { print; next }
    in_block { exit }
  ' "$1"
}

# get the row index of Displays
get_block_range() {
    awk '
    $0 ~ /^  Displays:/ { start=NR; next }
    start && $0 ~ /^   / { next }
    start { print start, NR-1; exit }
    END { if (start && !printed) print start, NR }
  ' "$1"
}

# get the scope of the block
read -r start_line end_line < <(get_block_range "$FILE_TARGET")

# create a temporary file
tmp_file=$(mktemp)

# replace the block
{
    head -n $((start_line - 1)) "$FILE_TARGET"
    extract_block "$FILE_REFERENCE"
    tail -n +"$((end_line + 1))" "$FILE_TARGET"
} >"$tmp_file"

# overwrite the file
mv "$tmp_file" "$FILE_TARGET"
echo "Updated 'Displays:' block of $FILE_TARGET"
