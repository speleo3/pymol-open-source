#!/bin/bash
#
# Build a self-contained PyMOL wheel for macOS (arm64).
#
# Third-party libraries (freetype, libpng, GLEW, netCDF) are built from
# source as static libraries with a fixed MACOSX_DEPLOYMENT_TARGET and linked
# into the extension module, so the wheel does not need any non-system dylib.
# OpenGL, zlib and libxml2 come from macOS. msgpack-cxx and mmtf-cpp are
# header-only.
#
# Usage:
#   macos/build-macos-wheel.sh [python-executable ...]
#
# Environment variables:
#   MACOSX_DEPLOYMENT_TARGET  minimum macOS version (default: 11.0)
#   WORKDIR                   build directory (default: build/macos-wheel)
#   JOBS                      parallel jobs (default: number of CPUs)
#   DIST_NAME                 distribution name of the wheel
#                             (default: pymol-open-source)
#   APP_NAME                  name of the macOS app launcher (default: PyMOL)
#
# Output: wheels in $WORKDIR/wheelhouse

set -euo pipefail

SRCDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKDIR="${WORKDIR:-$SRCDIR/build/macos-wheel}"
mkdir -p "$WORKDIR"
WORKDIR="$(cd "$WORKDIR" && pwd)"

export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-11.0}"
export JOBS="${JOBS:-$(sysctl -n hw.ncpu)}"
DIST_NAME="${DIST_NAME:-pymol-open-source}"
APP_NAME="${APP_NAME:-PyMOL}"
ARCH=arm64

if [[ $# -eq 0 ]]; then
    set -- python3
fi

DOWNLOADS="$WORKDIR/downloads"
DEPS_SRC="$WORKDIR/deps-src"
PREFIX="$WORKDIR/deps-$ARCH-$MACOSX_DEPLOYMENT_TARGET"
WHEELHOUSE="$WORKDIR/wheelhouse"
SDK="$(xcrun --sdk macosx --show-sdk-path)"

FREETYPE_VERSION=2.13.3
LIBPNG_VERSION=1.6.47
GLEW_VERSION=2.2.0
NETCDF_VERSION=4.9.3
MSGPACK_VERSION=6.1.1
MMTF_VERSION=1.1.0
GLM_VERSION=1.0.1

# name|url|sha256
SOURCES=(
    "freetype-$FREETYPE_VERSION.tar.xz|https://download.savannah.gnu.org/releases/freetype/freetype-$FREETYPE_VERSION.tar.xz|0550350666d427c74daeb85d5ac7bb353acba5f76956395995311a9c6f063289"
    "libpng-$LIBPNG_VERSION.tar.xz|https://download.sourceforge.net/libpng/libpng-$LIBPNG_VERSION.tar.xz|b213cb381fbb1175327bd708a77aab708a05adde7b471bc267bd15ac99893631"
    "glew-$GLEW_VERSION.tgz|https://github.com/nigels-com/glew/releases/download/glew-$GLEW_VERSION/glew-$GLEW_VERSION.tgz|d4fc82893cfb00109578d0a1a2337fb8ca335b3ceccf97b97e5cc7f08e4353e1"
    "netcdf-c-$NETCDF_VERSION.tar.gz|https://github.com/Unidata/netcdf-c/archive/refs/tags/v$NETCDF_VERSION.tar.gz|990f46d49525d6ab5dc4249f8684c6deeaf54de6fec63a187e9fb382cc0ffdff"
    "msgpack-cxx-$MSGPACK_VERSION.tar.gz|https://github.com/msgpack/msgpack-c/releases/download/cpp-$MSGPACK_VERSION/msgpack-cxx-$MSGPACK_VERSION.tar.gz|5fd555742e37bbd58d166199e669f01f743c7b3c6177191dd7b31fb0c37fa191"
    "mmtf-cpp-$MMTF_VERSION.tar.gz|https://github.com/rcsb/mmtf-cpp/archive/refs/tags/v$MMTF_VERSION.tar.gz|021173bdc1814b1d0541c4426277d39df2b629af53151999b137e015418f76c0"
    "glm-$GLM_VERSION.tar.gz|https://github.com/g-truc/glm/archive/refs/tags/$GLM_VERSION.tar.gz|9f3174561fd26904b23f0db5e560971cbf9b3cbda0b280f04d5c379d03bf234c"
)

log() { printf '\n==> %s\n' "$*"; }

# Compiler flags for all dependency builds
export CFLAGS="-arch $ARCH -O2 -fPIC"
export CXXFLAGS="$CFLAGS"
CMAKE_COMMON=(
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX="$PREFIX"
    -DCMAKE_INSTALL_LIBDIR=lib
    -DCMAKE_OSX_ARCHITECTURES=$ARCH
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$MACOSX_DEPLOYMENT_TARGET"
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DCMAKE_FIND_FRAMEWORK=LAST
    -DCMAKE_IGNORE_PREFIX_PATH="/opt/homebrew;/usr/local;/opt/local"
    -DBUILD_SHARED_LIBS=OFF
)

download_all() {
    mkdir -p "$DOWNLOADS"
    local entry name url sha
    for entry in "${SOURCES[@]}"; do
        IFS='|' read -r name url sha <<<"$entry"
        if [[ ! -f "$DOWNLOADS/$name" ]]; then
            log "Download $name"
            curl -fL --retry 3 -o "$DOWNLOADS/$name.part" "$url"
            mv "$DOWNLOADS/$name.part" "$DOWNLOADS/$name"
        fi
        if [[ -n "$sha" ]]; then
            echo "$sha  $DOWNLOADS/$name" | shasum -a 256 -c - >/dev/null
        fi
    done
}

# extract <archive> -> prints source directory
extract() {
    local archive="$DOWNLOADS/$1" dir="$DEPS_SRC/${1%.t*z}"
    rm -rf "$dir"
    mkdir -p "$dir"
    tar -xf "$archive" -C "$dir" --strip-components 1
    echo "$dir"
}

# save_license <src-dir> <file>... -> $PREFIX/licenses/<name>/
save_license() {
    local src="$1" dest="$PREFIX/licenses/$(basename "$1")"
    shift
    mkdir -p "$dest"
    local f
    for f in "$@"; do cp "$src/$f" "$dest/"; done
}

build_freetype() {
    local src
    src=$(extract "freetype-$FREETYPE_VERSION.tar.xz")
    cmake -S "$src" -B "$src/_build" "${CMAKE_COMMON[@]}" \
        -DFT_DISABLE_ZLIB=ON \
        -DFT_DISABLE_BZIP2=ON \
        -DFT_DISABLE_PNG=ON \
        -DFT_DISABLE_HARFBUZZ=ON \
        -DFT_DISABLE_BROTLI=ON
    cmake --build "$src/_build" -j "$JOBS"
    cmake --install "$src/_build"
    save_license "$src" LICENSE.TXT docs/FTL.TXT
}

build_libpng() {
    local src
    src=$(extract "libpng-$LIBPNG_VERSION.tar.xz")
    # zlib from the macOS SDK
    cmake -S "$src" -B "$src/_build" "${CMAKE_COMMON[@]}" \
        -DPNG_SHARED=OFF \
        -DPNG_STATIC=ON \
        -DPNG_FRAMEWORK=OFF \
        -DPNG_TESTS=OFF \
        -DPNG_TOOLS=OFF \
        -DZLIB_ROOT="$SDK/usr"
    cmake --build "$src/_build" -j "$JOBS"
    cmake --install "$src/_build"
    save_license "$src" LICENSE
}

build_glew() {
    local src
    src=$(extract "glew-$GLEW_VERSION.tgz")
    # GLEW's own build systems always produce a dylib next to the static
    # library. Compiling the single source file is simpler.
    mkdir -p "$PREFIX/lib" "$PREFIX/include/GL"
    cc $CFLAGS -mmacosx-version-min="$MACOSX_DEPLOYMENT_TARGET" \
        -DGLEW_STATIC -DGLEW_NO_GLU -I"$src/include" \
        -c "$src/src/glew.c" -o "$src/glew.o"
    rm -f "$PREFIX/lib/libGLEW.a"
    ar rcs "$PREFIX/lib/libGLEW.a" "$src/glew.o"
    cp "$src"/include/GL/{glew.h,glxew.h,wglew.h} "$PREFIX/include/GL/"
    save_license "$src" LICENSE.txt
}

build_netcdf() {
    local src
    src=$(extract "netcdf-c-$NETCDF_VERSION.tar.gz")
    # Classic and 64-bit offset formats only (no HDF5), which is what
    # the AMBER NetCDF trajectory plugin needs.
    cmake -S "$src" -B "$src/_build" "${CMAKE_COMMON[@]}" \
        -DNETCDF_ENABLE_NETCDF_4=OFF \
        -DNETCDF_ENABLE_HDF5=OFF \
        -DNETCDF_ENABLE_DAP=OFF \
        -DNETCDF_ENABLE_BYTERANGE=OFF \
        -DNETCDF_ENABLE_NCZARR=OFF \
        -DNETCDF_ENABLE_S3=OFF \
        -DNETCDF_ENABLE_LIBXML2=OFF \
        -DNETCDF_ENABLE_PLUGINS=OFF \
        -DNETCDF_ENABLE_FILTER_SZIP=OFF \
        -DNETCDF_ENABLE_FILTER_BZ2=OFF \
        -DNETCDF_ENABLE_FILTER_BLOSC=OFF \
        -DNETCDF_ENABLE_FILTER_ZSTD=OFF \
        -DNETCDF_ENABLE_TESTS=OFF \
        -DNETCDF_BUILD_UTILITIES=OFF \
        -DNETCDF_ENABLE_EXAMPLES=OFF \
        -DNETCDF_ENABLE_DOXYGEN=OFF
    cmake --build "$src/_build" -j "$JOBS"
    cmake --install "$src/_build"
    save_license "$src" COPYRIGHT
}

install_headers() {
    local src
    src=$(extract "msgpack-cxx-$MSGPACK_VERSION.tar.gz")
    cp -R "$src/include/." "$PREFIX/include/"
    save_license "$src" LICENSE_1_0.txt
    src=$(extract "mmtf-cpp-$MMTF_VERSION.tar.gz")
    cp -R "$src/include/." "$PREFIX/include/"
    save_license "$src" LICENSE
    src=$(extract "glm-$GLM_VERSION.tar.gz")
    cp -R "$src/glm" "$PREFIX/include/"
    rm -f "$PREFIX/include/glm/CMakeLists.txt"
    save_license "$src" copying.txt
}

build_deps() {
    local stamp="$PREFIX/.complete"
    # rebuild if any version or build recipe changes
    local stamp_content
    stamp_content="${SOURCES[*]} $MACOSX_DEPLOYMENT_TARGET $(declare -f \
        save_license build_freetype build_libpng build_glew build_netcdf \
        install_headers | shasum -a 256)"
    if [[ -f "$stamp" && "$(cat "$stamp")" == "$stamp_content" ]]; then
        log "Dependencies already built in $PREFIX"
        return
    fi
    rm -rf "$PREFIX"
    mkdir -p "$PREFIX"
    download_all
    log "Build freetype";  build_freetype
    log "Build libpng";    build_libpng
    log "Build GLEW";      build_glew
    log "Build netCDF";    build_netcdf
    log "Install headers"; install_headers

    # Make sure the linker can only pick the static libraries
    find "$PREFIX/lib" -name '*.dylib' -delete
    echo "$stamp_content" > "$stamp"
}

# Copy the working tree to a clean directory, so that stale CMake caches in
# ./build cannot leak into the wheel.
stage_source() {
    local stage="$1"
    rm -rf "$stage"
    mkdir -p "$stage"
    (
        cd "$SRCDIR"
        git ls-files -z --cached --others --exclude-standard -- . \
                ':(exclude)build' ':(exclude)_old' \
                ':(exclude).venv*' ':(exclude)*.egg-info' |
            tar --null -T - -cf -
    ) | tar -xf - -C "$stage"
    [[ -f "$stage/setup.py" ]] || { echo "ERROR: staging failed" >&2; exit 1; }
    # Set the distribution name in the staged copy only. The import
    # package name ("pymol") does not change.
    sed -i '' -E "1,/^name = /s/^name = \".*\"$/name = \"$DIST_NAME\"/" \
        "$stage/pyproject.toml"
    grep -q -x "name = \"$DIST_NAME\"" "$stage/pyproject.toml" ||
        { echo "ERROR: failed to set name in pyproject.toml" >&2; exit 1; }
    # Add fork suffix to version
    sed -i '' -E '1,/^#define _PyMOL_VERSION /s/(\+.*)?"$/+thomas"/' \
        "$stage/layer0/Version.h"
    # git SHA for PyMOLBuildInfo.h
    ln -s "$SRCDIR/.git" "$stage/.git"
}

# Build the app launcher bundle (see macos/launcher.m). It does not know
# the Python interpreter yet. "python -m pymol.macos_app install" copies it
# to ~/Applications and links it to the current environment.
build_app() {
    local stage="$1" app="$2"
    rm -rf "$app"
    mkdir -p "$app/Contents/MacOS"
    cp -R "$stage/macos/Contents/Resources" "$app/Contents/"
    sed "s/%APPNAME%/$APP_NAME/g" "$stage/macos/Contents/Info.plist" \
        > "$app/Contents/Info.plist"
    plutil -replace LSMinimumSystemVersion \
        -string "$MACOSX_DEPLOYMENT_TARGET" "$app/Contents/Info.plist"
    plutil -lint "$app/Contents/Info.plist" >/dev/null
    clang -arch "$ARCH" -mmacosx-version-min="$MACOSX_DEPLOYMENT_TARGET" \
        -O2 -fobjc-arc -framework Cocoa \
        "$stage/macos/launcher.m" -o "$app/Contents/MacOS/$APP_NAME"
}

# Add files to the wheel:
# - the app launcher as pymol/macos/<name>.app
# - the licenses of the statically linked libraries in
#   .dist-info/licenses/thirdparty
add_extras() {
    local python="$1" whl="$2" tmp distinfo
    local app="$3"
    tmp=$(mktemp -d "$WORKDIR/extras.XXXXXX")
    "$python" -m wheel unpack -d "$tmp" "$whl" >/dev/null
    distinfo=$(echo "$tmp"/*/*.dist-info)
    mkdir -p "$distinfo/licenses/thirdparty"
    cp -R "$PREFIX/licenses/." "$distinfo/licenses/thirdparty/"
    mkdir -p "$(dirname "$distinfo")/pymol/macos"
    cp -R "$app" "$(dirname "$distinfo")/pymol/macos/"
    rm "$whl"
    "$python" -m wheel pack -d "$(dirname "$whl")" "$(dirname "$distinfo")" >/dev/null
    rm -rf "$tmp"
}

check_wheel() {
    local whl="$1" tmp bad=0 so
    tmp=$(mktemp -d "$WORKDIR/check.XXXXXX")
    unzip -q "$whl" -d "$tmp"
    while IFS= read -r so; do
        echo "${so#$tmp/}:"
        # dependencies, without the module's own install name (LC_ID_DYLIB)
        local deps
        deps=$(otool -L "$so" | tail -n +2 | awk '{print $1}' |
            grep -v -F -x "$(otool -D "$so" | tail -n +2)" || true)
        echo "$deps" | sed 's/^/   /'
        if echo "$deps" | grep -v -E '^(/usr/lib/|/System/Library/)'; then
            echo "ERROR: non-system library dependency in ${so#$tmp/}" >&2
            bad=1
        fi
        local minos
        minos=$(otool -l "$so" | awk '/LC_BUILD_VERSION/{f=1} f&&/minos/{print $2; exit}')
        echo "   minos: $minos, archs: $(lipo -archs "$so")"
    done < <(find "$tmp" -name '*.so' -o -path '*.app/Contents/MacOS/*' -type f)
    echo "app launcher:"
    (cd "$tmp" && ls -d -- pymol/macos/*.app | sed 's/^/   /')
    echo "third-party licenses:"
    (cd "$tmp" && ls -d -- */licenses/thirdparty/* | sed 's/^/   /')
    rm -rf "$tmp"
    return $bad
}

build_wheel() {
    local python="$1" stage="$2" venv pyver
    pyver=$("$python" -c 'import sys; print("%d.%d" % sys.version_info[:2])')
    venv="$WORKDIR/venv-$pyver"
    log "Build wheel with $python (Python $pyver)"

    if [[ ! -x "$venv/bin/python" ]]; then
        "$python" -m venv "$venv"
    fi
    "$venv/bin/python" -m pip install -q --upgrade pip build delocate wheel

    local dist="$WORKDIR/dist-$pyver"
    rm -rf "$dist" "$stage/build"

    (
        cd "$stage"
        export PREFIX_PATH="$PREFIX"
        # CMake reads these at configure time
        # (GLM_ENABLE_EXPERIMENTAL: layer2/CifMoleculeReader.cpp uses glm/gtx)
        export CFLAGS="-arch $ARCH -I$SDK/usr/include/libxml2 -DGLM_ENABLE_EXPERIMENTAL"
        export CXXFLAGS="$CFLAGS"
        # static libpng needs zlib
        export LDFLAGS="-arch $ARCH -lz"
        export ARCHFLAGS="-arch $ARCH"
        "$venv/bin/python" -m build --wheel --outdir "$dist" \
            -Cuse-msgpackc=c++11 \
            -Clibxml=yes \
            -Cvmd-plugins=yes \
            -Cglut=no \
            -Cuse-openmp=no \
            -Ctesting=no \
            .
    )

    # delocate retags the wheel (e.g. macosx_26_0 -> macosx_11_0) based on
    # the minimum macOS version of the binaries
    local delocated="$WORKDIR/delocated-$pyver" whl
    rm -rf "$delocated"
    for whl in "$dist"/*.whl; do
        log "delocate $(basename "$whl")"
        "$venv/bin/delocate-listdeps" --all "$whl"
        "$venv/bin/delocate-wheel" -v -w "$delocated" \
            --require-archs "$ARCH" \
            --require-target-macos-version "$MACOSX_DEPLOYMENT_TARGET" \
            "$whl"
    done

    mkdir -p "$WHEELHOUSE"
    for whl in "$delocated"/*.whl; do
        add_extras "$venv/bin/python" "$whl" "$APP"
        log "Check $(basename "$whl")"
        check_wheel "$whl"
        mv "$whl" "$WHEELHOUSE/"
    done
}

build_deps
STAGE="$WORKDIR/src"
log "Stage source tree in $STAGE"
stage_source "$STAGE"
APP="$WORKDIR/app/$APP_NAME.app"
log "Build $APP_NAME.app"
build_app "$STAGE" "$APP"
for python in "$@"; do
    build_wheel "$python" "$STAGE"
done

log "Done. Wheels:"
ls -l "$WHEELHOUSE"
