#!/usr/bin/env bash

set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$script_dir/.." && pwd)
label=${1:-test}
safe_label=$(printf '%s' "$label" | tr -cd 'A-Za-z0-9_-')

if [ -z "$safe_label" ]; then
    safe_label=test
fi

timestamp=$(date '+%Y-%m-%d_%H-%M-%S')
run_dir="$script_dir/runs/${timestamp}_${safe_label}"

mkdir -p "$run_dir"
cp "$script_dir/templates/telemetry.csv" "$run_dir/telemetry.csv"
: > "$run_dir/telemetry.raw"
cp "$script_dir/templates/params.json" "$run_dir/params.json"
cp "$script_dir/templates/events.log" "$run_dir/events.log"
cp "$script_dir/templates/notes.md" "$run_dir/notes.md"

commit=$(git -C "$repo_dir" rev-parse HEAD 2>/dev/null || printf 'unknown')
branch=$(git -C "$repo_dir" branch --show-current 2>/dev/null || printf 'unknown')
dirty=$(git -C "$repo_dir" status --short -- . 2>/dev/null || true)

{
    printf 'created_at=%s\n' "$(date --iso-8601=seconds)"
    printf 'git_commit=%s\n' "$commit"
    printf 'git_branch=%s\n' "$branch"
    if [ -n "$dirty" ]; then
        printf 'worktree_dirty=yes\n'
        printf '%s\n' "$dirty"
    else
        printf 'worktree_dirty=no\n'
    fi
} > "$run_dir/firmware.txt"

printf '%s\n' "$run_dir"
