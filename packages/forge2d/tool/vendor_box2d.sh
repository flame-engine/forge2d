#!/usr/bin/env bash
# Refreshes the vendored Box2D snapshot in third_party/box2d from an upstream tag.
#
# Usage: tool/vendor_box2d.sh v3.1.1
#
# After running, regenerate the bindings (dart run ffigen) and update the
# source list in hook/build.dart if files were added or removed upstream.
set -euo pipefail

TAG="${1:?usage: tool/vendor_box2d.sh <tag, e.g. v3.1.1>}"
PACKAGE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$PACKAGE_DIR/third_party/box2d"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Downloading box2d $TAG..."
curl -sL "https://github.com/erincatto/box2d/archive/refs/tags/$TAG.tar.gz" \
  | tar xz -C "$TMP" --strip-components=1

SHA="$(git ls-remote https://github.com/erincatto/box2d.git "refs/tags/$TAG" | cut -f1)"

rm -rf "$DEST"
mkdir -p "$DEST/include/box2d" "$DEST/src"
cp "$TMP"/include/box2d/*.h "$DEST/include/box2d/"
cp "$TMP"/src/*.c "$TMP"/src/*.h "$DEST/src/"
rm -f "$DEST/src/box2d.natvis"
cp "$TMP/LICENSE" "$DEST/LICENSE"
printf '%s\nupstream: https://github.com/erincatto/box2d\ntag-sha: %s\n' \
  "$TAG" "$SHA" > "$DEST/VERSION"

echo "Vendored box2d $TAG ($SHA) into third_party/box2d"
echo "Source files:"
ls "$DEST/src"/*.c | wc -l
echo "Next steps: update hook/build.dart source list if needed, dart run ffigen"
