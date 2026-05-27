#!/bin/bash
# =============================================================================
# setup_macos.sh — macOS environment setup for Differential Drive Mobile Robot
#
# What this script does:
#   1. Checks macOS prerequisites
#   2. Installs Miniconda (if not already present)
#   3. Installs Gazebo Sim Harmonic via Homebrew
#   4. Creates the ros2_jazzy conda environment via RoboStack
#   5. Installs Python dependencies via pip install -e (pyproject.toml)
#   6. Builds the workspace with colcon
#   7. Configures your ~/.zshrc for auto-activation
#
# Usage:
#   chmod +x setup_macos.sh
#   ./setup_macos.sh
#
# Tested on: macOS Tahoe 26.5 (Apple Silicon)
# =============================================================================

set -euo pipefail

# Filter out broken-plugin noise from conda (anaconda-auth / pydantic_core)
exec 2> >(grep -v "Error while loading conda entry point" >&2)

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${BLUE}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*"; exit 1; }
step()    { echo -e "\n${BOLD}━━━ $* ━━━${RESET}"; }

# ── Config ────────────────────────────────────────────────────────────────────
CONDA_ENV="ros2_jazzy"
ROS_DISTRO="jazzy"
WORKSPACE="$HOME/ros2_ws"
MINICONDA_DIR="$HOME/miniconda3"
MINICONDA_INSTALLER="/tmp/miniconda_installer.sh"

# ── Step 0: macOS check ───────────────────────────────────────────────────────
step "Step 0 — Checking system"

if [[ "$(uname)" != "Darwin" ]]; then
    error "This script is macOS only. For Ubuntu, use setup_ubuntu.sh."
fi

MACOS_VERSION=$(sw_vers -productVersion)
ARCH=$(uname -m)
info "macOS $MACOS_VERSION on $ARCH"

if [[ "$ARCH" == "arm64" ]]; then
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh"
else
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh"
fi

success "System check passed"

# ── Step 1: Miniconda ─────────────────────────────────────────────────────────
step "Step 1 — Miniconda"

if [ -d "$MINICONDA_DIR" ] || command -v conda &>/dev/null; then
    success "Miniconda already installed — skipping"
else
    info "Downloading Miniconda for $ARCH..."
    curl -fsSL "$MINICONDA_URL" -o "$MINICONDA_INSTALLER"
    bash "$MINICONDA_INSTALLER" -b -p "$MINICONDA_DIR"
    rm -f "$MINICONDA_INSTALLER"
    success "Miniconda installed at $MINICONDA_DIR"
fi

# Make conda available in this shell session
export PATH="$MINICONDA_DIR/bin:$PATH"
# shellcheck disable=SC1091
source "$MINICONDA_DIR/etc/profile.d/conda.sh"

# ── Step 2: Gazebo Sim Harmonic ───────────────────────────────────────────────
step "Step 2 — Gazebo Sim Harmonic"

if command -v gz &>/dev/null && gz sim --version 2>/dev/null | grep -q "Harmonic"; then
    success "Gazebo Sim Harmonic already installed — skipping"
else
    info "Tapping osrf/simulation..."
    brew tap osrf/simulation
    brew update
    info "Installing gz-harmonic (this may take a few minutes)..."
    brew install gz-harmonic
    success "Gazebo Sim Harmonic installed"
fi

# ── Step 3: Conda channel config ─────────────────────────────────────────────
step "Step 3 — Conda channels"

info "Configuring conda channels (conda-forge + robostack-jazzy)..."
conda config --set channel_priority strict
conda config --add channels conda-forge
conda config --add channels robostack-jazzy
conda config --remove channels defaults 2>/dev/null || true
conda config --add channels nodefaults 2>/dev/null || true
success "Channels configured"

# ── Step 4: Create ROS2 conda environment ────────────────────────────────────
step "Step 4 — conda env: $CONDA_ENV"

if conda env list | grep -q "^${CONDA_ENV}"; then
    success "Environment '$CONDA_ENV' already exists — skipping creation"
else
    info "Creating conda environment '$CONDA_ENV' with ROS2 $ROS_DISTRO..."
    info "This may take 5–15 minutes on first install..."
    conda create -n "$CONDA_ENV" \
        --channel robostack-jazzy \
        --channel conda-forge \
        -y --quiet \
        python=3.12 \
        "ros-${ROS_DISTRO}-desktop" \
        "ros-${ROS_DISTRO}-nav2-bringup" \
        "ros-${ROS_DISTRO}-ros-gz" \
        "ros-${ROS_DISTRO}-ros-gz-bridge" \
        "ros-${ROS_DISTRO}-ros-gz-sim" \
        "ros-${ROS_DISTRO}-slam-toolbox" \
        "ros-${ROS_DISTRO}-robot-state-publisher" \
        "ros-${ROS_DISTRO}-joint-state-publisher" \
        "ros-${ROS_DISTRO}-joint-state-broadcaster" \
        "ros-${ROS_DISTRO}-ros2-controllers" \
        "ros-${ROS_DISTRO}-rosbridge-suite" \
        colcon-common-extensions \
        colcon-ros
    success "Environment '$CONDA_ENV' created"
fi

# ── Step 5: Python extras (pyproject.toml deps) ───────────────────────────────
step "Step 5 — Python extras"

info "Installing project dependencies from pyproject.toml (editable mode)..."
conda run -n "$CONDA_ENV" pip install -e "$WORKSPACE/src/Differential_Drive_Mobile_Robot" --quiet
success "Python extras installed"

# ── Step 6: Build workspace ───────────────────────────────────────────────────
step "Step 6 — colcon build"

info "Cleaning previous build artifacts..."
rm -rf "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"

info "Building the workspace..."
conda run -n "$CONDA_ENV" bash -c "
    source \"\$CONDA_PREFIX/setup.sh\"
    cd $WORKSPACE
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_cohesion+
"
success "Workspace built successfully"

# ── Step 7: Configure ~/.zshrc ───────────────────────────────────────────────
step "Step 7 — Shell config (~/.zshrc)"

ZSHRC="$HOME/.zshrc"
MARKER="# >>> ros2_jazzy workspace setup >>>"
MARKER_END="# <<< ros2_jazzy workspace setup <<<"

if grep -q "$MARKER" "$ZSHRC" 2>/dev/null; then
    success ".zshrc already configured — skipping"
else
    info "Adding ROS2 environment setup to ~/.zshrc..."
    cat >> "$ZSHRC" << EOF

$MARKER
# Added by setup_macos.sh — Differential Drive Mobile Robot
conda activate $CONDA_ENV
source ~/ros2_ws/install/setup.bash
$MARKER_END
EOF
    success ".zshrc updated"
fi

# ── Done ──────────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}${BOLD}╔══════════════════════════════════════════════════════╗${RESET}"
echo -e "${GREEN}${BOLD}║        Setup complete! 🎉                            ║${RESET}"
echo -e "${GREEN}${BOLD}╚══════════════════════════════════════════════════════╝${RESET}"
echo ""
echo -e "  Workspace : ${BOLD}$WORKSPACE${RESET}"
echo -e "  Env       : ${BOLD}conda activate $CONDA_ENV${RESET}"
echo -e "  Build dir : ${BOLD}$WORKSPACE/install${RESET}"
echo ""
echo -e "  ${YELLOW}Next step:${RESET} Open a new terminal (or run ${BOLD}source ~/.zshrc${RESET})"
echo -e "  Then test with: ${BOLD}ros2 topic list${RESET}"
echo ""
