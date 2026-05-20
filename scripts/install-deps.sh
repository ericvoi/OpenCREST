#!/usr/bin/env bash
# install-deps.sh — Install system dependencies for OpenCREST
#
# Usage:
#   sudo ./scripts/install-deps.sh
#
# Supports: Ubuntu/Debian, Fedora/RHEL, Arch Linux
#
# The following libraries are fetched automatically by CMake (FetchContent)
# and do NOT need to be installed manually:
#   - yaml-cpp 0.8.0
#   - spdlog 1.14.1
#   - Google Test 1.14.0 (tests only)

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: run with sudo or as root." >&2
    exit 1
fi

install_debian() {
    apt-get update
    apt-get install -y \
        build-essential \
        cmake \
        pkg-config \
        libusb-1.0-0-dev \
        git
}

install_fedora() {
    dnf install -y \
        gcc-c++ \
        cmake \
        pkgconf-pkg-config \
        libusb1-devel \
        git
}

install_arch() {
    pacman -Sy --noconfirm --needed \
        base-devel \
        cmake \
        pkgconf \
        libusb \
        git
}

if [ -f /etc/os-release ]; then
    . /etc/os-release
    case "$ID" in
        ubuntu|debian|linuxmint|pop)
            echo "Detected $PRETTY_NAME — using apt"
            install_debian
            ;;
        fedora|rhel|centos|rocky|alma)
            echo "Detected $PRETTY_NAME — using dnf"
            install_fedora
            ;;
        arch|manjaro|endeavouros)
            echo "Detected $PRETTY_NAME — using pacman"
            install_arch
            ;;
        *)
            echo "Unsupported distribution: $ID" >&2
            echo ""
            echo "Install the following packages manually:"
            echo "  - C++20 compiler (GCC >= 11 or Clang >= 14)"
            echo "  - CMake >= 3.20"
            echo "  - pkg-config"
            echo "  - libusb-1.0 development headers"
            echo "  - git"
            exit 1
            ;;
    esac
else
    echo "Cannot detect distribution (/etc/os-release not found)." >&2
    exit 1
fi

echo ""
echo "System dependencies installed. Build with:"
echo "  cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo"
echo "  cmake --build build -j\$(nproc)"
