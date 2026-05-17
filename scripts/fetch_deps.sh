#!/bin/bash
set -e

# Fetch all external dependencies for kigstudio
# Run this from the project root directory

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "=========================================="
echo "Fetching dependencies for kigstudio"
echo "Root: $ROOT"
echo "=========================================="

# 1. Update Git submodules
echo ""
echo "[1/4] Updating Git submodules..."
cd "$ROOT"
git submodule update --init --recursive

# 2. Download Eigen3 (header-only)
echo ""
echo "[2/4] Checking Eigen3..."
if [ -f "$ROOT/dep/eigen3/Eigen/Core" ]; then
    echo "Eigen3 already exists, skipping."
else
    echo "Downloading Eigen3 3.4.0..."
    rm -rf "$ROOT/dep/eigen3"
    curl -L -o "$ROOT/dep/eigen-3.4.0.tar.gz" "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"
    tar -xzf "$ROOT/dep/eigen-3.4.0.tar.gz" -C "$ROOT/dep"
    mv "$ROOT/dep/eigen-3.4.0" "$ROOT/dep/eigen3"
    rm "$ROOT/dep/eigen-3.4.0.tar.gz"
    echo "Eigen3 ready."
fi

# 3. Download Boost and generate headers
echo ""
echo "[3/4] Checking Boost..."
if [ -f "$ROOT/dep/boost/boost/version.hpp" ]; then
    echo "Boost already exists, skipping."
else
    echo "Downloading Boost 1.82.0..."
    rm -rf "$ROOT/dep/boost"
    curl -L -o "$ROOT/dep/boost_1_82_0.tar.gz" "https://github.com/boostorg/boost/releases/download/boost-1.82.0/boost-1.82.0.tar.gz"
    tar -xzf "$ROOT/dep/boost_1_82_0.tar.gz" -C "$ROOT/dep"
    mv "$ROOT/dep/boost-1.82.0" "$ROOT/dep/boost"
    rm "$ROOT/dep/boost_1_82_0.tar.gz"

    echo "Running Boost bootstrap..."
    cd "$ROOT/dep/boost"
    ./bootstrap.sh

    echo "Generating Boost headers..."
    ./b2 headers
    cd "$ROOT"
    echo "Boost ready."
fi

# 4. Verify CGAL (should already be present as a submodule)
echo ""
echo "[4/4] Checking CGAL..."
if [ -f "$ROOT/dep/cgal/Installation/CGALConfig.cmake" ]; then
    echo "CGAL found."
else
    echo "WARNING: CGAL not found. Make sure the submodule is initialized."
    echo "  git submodule update --init --recursive dep/cgal"
fi

echo ""
echo "=========================================="
echo "All dependencies fetched successfully!"
echo "=========================================="
