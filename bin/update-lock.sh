#!/usr/bin/env bash
# Regenerate conan.lock. Run it after changing a requirement in conanfile.py.
#
# The lock starts from the conan.lock that viam-cpp-sdk attaches to each release, so the SDK and its
# dependencies resolve to the exact versions viamconan has prebuilt binaries for. Without it, open
# version ranges drift to newer releases on conancenter and everything builds from source. The rest
# of our dependencies are then resolved for each platform we ship, and the results are merged.
#
# Needs conan (see bin/setup.sh) and curl.
set -euo pipefail
cd "$(dirname "$0")/.."

SDK_VERSION=$(sed -nE 's/.*requires\("viam-cpp-sdk\/([^"]+)"\).*/\1/p' conanfile.py)
if [[ ! "${SDK_VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Could not read a single viam-cpp-sdk version from conanfile.py (got: '${SDK_VERSION}')" >&2
  exit 1
fi

tmp=$(mktemp -d)
trap 'rm -rf "${tmp}"' EXIT

curl -fsSL -o "${tmp}/sdk.lock" \
  "https://github.com/viamrobotics/viam-cpp-sdk/releases/download/releases/v${SDK_VERSION}/conan.lock"

# Settings for the platforms we ship. Linux matches the cpp-sdk-conan-ubuntu:jammy default profile,
# macOS is etc/conan/macos.profile, and Windows matches the windows-2022 runner.
cat > "${tmp}/linux" <<'EOF'
[settings]
os=Linux
compiler=gcc
compiler.version=11
compiler.libcxx=libstdc++11
compiler.cppstd=17
build_type=Release
EOF
cat > "${tmp}/windows" <<'EOF'
[settings]
os=Windows
arch=x86_64
compiler=msvc
compiler.version=194
compiler.runtime=dynamic
compiler.runtime_type=Release
compiler.cppstd=17
build_type=Release
EOF

# etc/conan/macos.profile includes the default profile.
conan profile detect --exist-ok >/dev/null

lock_for() {
  local name=$1
  shift
  conan lock create . "$@" \
    -o:a "viam-cpp-sdk/*:shared=False" \
    --lockfile "${tmp}/sdk.lock" --lockfile-partial --lockfile-clean \
    --lockfile-out "${tmp}/${name}.lock"
}

lock_for linux-amd64 -pr:a "${tmp}/linux" -s:a arch=x86_64
lock_for linux-arm64 -pr:a "${tmp}/linux" -s:a arch=armv8
lock_for darwin-arm64 -pr:a etc/conan/macos.profile
lock_for windows-amd64 -pr:a "${tmp}/windows"

conan lock merge \
  --lockfile "${tmp}/linux-amd64.lock" \
  --lockfile "${tmp}/linux-arm64.lock" \
  --lockfile "${tmp}/darwin-arm64.lock" \
  --lockfile "${tmp}/windows-amd64.lock" \
  --lockfile-out conan.lock
