#!/usr/bin/env bash
# Fast fail the script on failures.
set -e

# Run the benchmark script as a simple sanity check
dart benchmark/web/bench2d.dart
