#!/bin/bash
# Builds build/EMU0202.driver (Core Audio plug-in) and build/emutest (engine test tool).
set -euo pipefail
cd "$(dirname "$0")"

CFLAGS=(-arch arm64 -O2 -Wall -Wextra -Wno-unused-parameter -mmacosx-version-min=13.0)
FRAMEWORKS=(-framework CoreAudio -framework CoreFoundation -framework Foundation -framework IOKit -framework IOUSBHost)
OUT=build/EMU0202.driver

rm -rf build
mkdir -p "$OUT/Contents/MacOS" build/obj
cp src/Info.plist "$OUT/Contents/Info.plist"

clang "${CFLAGS[@]}" -fvisibility=hidden -c src/EMU0202Driver.c -o build/obj/EMU0202Driver.o
clang "${CFLAGS[@]}" -fvisibility=hidden -fobjc-arc -c src/EMUEngine.m -o build/obj/EMUEngine.o
clang "${CFLAGS[@]}" -bundle build/obj/EMU0202Driver.o build/obj/EMUEngine.o -o "$OUT/Contents/MacOS/EMU0202" "${FRAMEWORKS[@]}"
codesign --force --sign - "$OUT"

clang "${CFLAGS[@]}" -fobjc-arc tools/emutest.m src/EMUEngine.m -o build/emutest "${FRAMEWORKS[@]}"
clang "${CFLAGS[@]}" tools/plugintest.c -o build/plugintest -framework CoreAudio -framework CoreFoundation
clang "${CFLAGS[@]}" tools/haltest.c -o build/haltest -framework CoreAudio -framework CoreFoundation
clang "${CFLAGS[@]}" tools/controltest.c -o build/controltest -framework CoreAudio -framework CoreFoundation
clang "${CFLAGS[@]}" tools/looptest.c -o build/looptest -framework CoreAudio -framework CoreFoundation

echo "Built $OUT, build/emutest, build/plugintest, build/haltest, build/controltest, build/looptest"
