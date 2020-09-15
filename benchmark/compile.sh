#!/bin/bash -e

shopt -s nullglob
for f in *.dart; do
  if [ "$f" == "demo.dart" ] ; then
    continue;
  fi
  dart2js -o $f.js $f
done
