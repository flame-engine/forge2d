#!/usr/bin/env bash

dartfmt --set-exit-if-changed -n .
if [ $? -eq 1 ]; then
  echo "dart format issues (please run 'dartfmt -w .')"
  exit 1
fi

pub get
result=$(pub run dart_code_metrics:metrics .)
if [ "$result" != "" ]; then
  echo "Dart code metrics issues:"
  echo "$result"
  # TODO: fail here once current issues are fixes
fi

dart analyze .
if [ $? -ne 0 ]; then
  echo "Found dart analyze issues"
  # TODO: fail here once current issues are fixes
fi

echo "No issues found in code!"
