#!/usr/bin/env bash
FORMAT_ISSUES=$(dartfmt --set-exit-if-changed -n .)
if [ $? -eq 1 ]; then
  echo "dart format issues on: (please run 'dartfmt -w .'"
  echo $FORMAT_ISSUES
  exit 1
fi

pub get
result=$(dartanalyzer .)
if ! echo "$result" | grep -q "No issues found!"; then
  echo "$result"
  echo "dart analyze issue: $1"
  exit 1
fi

echo "success"
exit 0
