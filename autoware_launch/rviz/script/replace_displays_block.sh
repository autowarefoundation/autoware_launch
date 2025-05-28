#!/bin/bash

FILE_REFERENCE="$1"
FILE_TARGET="$2"

# Displays ブロックを抽出（2スペースインデント想定）
extract_block() {
  awk '
    $0 ~ /^  Displays:/ { in_block=1; print; next }
    in_block && $0 ~ /^   / { print; next }
    in_block { exit }
  ' "$1"
}

# Displays ブロックの行番号範囲を取得
get_block_range() {
  awk '
    $0 ~ /^  Displays:/ { start=NR; next }
    start && $0 ~ /^   / { next }
    start { print start, NR-1; exit }
    END { if (start && !printed) print start, NR }
  ' "$1"
}

# ブロック範囲取得
read start_line end_line < <(get_block_range "$FILE_TARGET")

# 一時ファイル作成
tmp_file=$(mktemp)

# 置換処理
{
  head -n $((start_line - 1)) "$FILE_TARGET"
  extract_block "$FILE_REFERENCE"
  tail -n +"$((end_line + 1))" "$FILE_TARGET"
} > "$tmp_file"

# 上書き or 出力
mv "$tmp_file" "$FILE_TARGET"
echo "Updated 'Displays:' block of $FILE_TARGET"
