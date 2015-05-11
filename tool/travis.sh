#!/bin/bash

# Fast fail the script on failures.   
set -e

# Only check formatting on stable branch
if [ "$TRAVIS_DART_VERSION" = "stable" ]; then
  $(dirname -- "$0")/ensure_dartfmt.sh
fi

dartanalyzer lib/box2d.dart

# Run the benchmark script as a simple sanity check
dart --checked --error_on_bad_type --error_on_bad_override benchmark/bench2d.dart

# Install dart_coveralls; gather and send coverage data.
if [ "$COVERALLS_TOKEN" ] && [ "$TRAVIS_DART_VERSION" = "stable" ]; then
  pub global activate dart_coveralls
  pub global run dart_coveralls report \
    --token $COVERALLS_TOKEN \
    --retry 2 \
    --exclude-test-files \
    benchmark/bench2d.dart
fi
