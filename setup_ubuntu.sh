#!/bin/bash
# =============================================================================
# setup_ubuntu.sh — Ubuntu 24.04 environment setup for Differential Drive Mobile Robot
#
# What this script does:
#   1. Checks Ubuntu 24.04 (Noble) prerequisites
#   2. Configures locale + Universe repo
#   3. Adds ROS 2 apt source and installs ROS 2 Jazzy (desktop + Nav2 + SLAM + Gazebo)
#   4. Installs Miniconda (if not already present)
#   5. Creates the ros2_jazzy conda environment (Python 3.12, matches system ABI)
#   6. Installs Python dependencies via pip install -e (pyproject.toml)
#   7. Builds the workspace with colcon
#   8. Configures your ~/.bashrc for auto-activation
#
# Usage:
#   chmod +x setup_ubuntu.sh
#   ./setup_ubuntu.sh
#
# Note: requires sudo (you'll be prompted for your password).
# Tested on: Ubuntu 24.04 LTS (Noble Numbat)
# =============================================================================

set -euo pipefail

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${BLUE}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*"; exit 1; }
step()    { echo -e "\n${BOLD}━━━ $* ━━━${RESET}"; }

# ── Config ────────────────────────────────────────────────────────────────────
ROS_DISTRO="jazzy"
WORKSPACE="$HOME/ros2_ws"
PROJECT_DIR="$WORKSPACE/src/Differential_Drive_Mobile_Robot"
CONDA_ENV="ros2_jazzy"
MINICONDA_DIR="$HOME/miniconda3"
MINICONDA_INSTALLER="/tmp/miniconda_installer.sh"

# ── Step 0: Ubuntu check ──────────────────────────────────────────────────────
step "Step 0 — Checking system"

if [[ "$(uname)" != "Linux" ]]; then
    error "This script is Linux only. For macOS, use setup_macos.sh."
fi

if [[ ! -f /etc/os-release ]]; then
    error "Cannot detect OS — /etc/os-release missing."
fi

# shellcheck disable=SC1091
. /etc/os-release

if [[ "${ID:-}" != "ubuntu" ]]; then
    error "This script targets Ubuntu (detected: ${ID:-unknown}). For macOS, use setup_macos.sh."
fi

if [[ "${VERSION_ID:-}" != "24.04" ]]; then
    warn "This script is tested on Ubuntu 24.04 (Noble). Detected ${VERSION_ID:-unknown} — continuing, but YMMV."
fi

ARCH=$(dpkg --print-architecture)
info "Ubuntu ${VERSION_ID} (${VERSION_CODENAME}) on ${ARCH}"

case "$(uname -m)" in
    x86_64)  MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh" ;;
    aarch64) MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh" ;;
    *)       error "Unsupported architecture: $(uname -m)" ;;
esac

if ! sudo -v; then
    error "sudo is required to install system packages."
fi

success "System check passed"

# ── Step 1: Locale + Universe ─────────────────────────────────────────────────
step "Step 1 — Locale & Universe repository"

if locale | grep -q "LANG=.*UTF-8"; then
    success "UTF-8 locale already active — skipping"
else
    info "Setting UTF-8 locale..."
    sudo apt update
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    success "Locale configured"
fi

info "Ensuring Ubuntu Universe repo is enabled..."
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
success "Universe enabled"

# ── Step 2: ROS 2 apt source ──────────────────────────────────────────────────
step "Step 2 — ROS 2 apt source"

if dpkg -l ros2-apt-source &>/dev/null; then
    success "ros2-apt-source already installed — skipping"
else
    info "Installing ROS 2 apt source package..."
    sudo apt update
    sudo apt install -y curl ca-certificates
    ROS_APT_SOURCE_VERSION=$(
        curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F\" '{print $4}'
    )
    if [[ -z "${ROS_APT_SOURCE_VERSION:-}" ]]; then
        error "Could not resolve latest ros-apt-source release tag."
    fi
    info "Latest ros-apt-source: ${ROS_APT_SOURCE_VERSION}"
    DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
    curl -fsSL "$DEB_URL" -o /tmp/ros2-apt-source.deb
    sudo apt install -y /tmp/ros2-apt-source.deb
    rm -f /tmp/ros2-apt-source.deb
    success "ROS 2 apt source configured"
fi

# ── Step 3: Install ROS 2 Jazzy + project packages ────────────────────────────
step "Step 3 — apt install: ROS 2 ${ROS_DISTRO} + deps"

info "Refreshing apt index..."
sudo apt update

info "Installing ROS 2 ${ROS_DISTRO} and project packages (this may take several minutes)..."
sudo apt install -y \
    "ros-${ROS_DISTRO}-desktop" \
    ros-dev-tools \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-nav2-bringup" \
    "ros-${ROS_DISTRO}-slam-toolbox" \
    "ros-${ROS_DISTRO}-ros-gz" \
    "ros-${ROS_DISTRO}-ros-gz-bridge" \
    "ros-${ROS_DISTRO}-ros-gz-sim" \
    "ros-${ROS_DISTRO}-robot-state-publisher" \
    "ros-${ROS_DISTRO}-joint-state-publisher" \
    "ros-${ROS_DISTRO}-joint-state-publisher-gui" \
    "ros-${ROS_DISTRO}-joint-state-broadcaster" \
    "ros-${ROS_DISTRO}-ros2-controllers" \
    "ros-${ROS_DISTRO}-ros2-control" \
    "ros-${ROS_DISTRO}-rosbridge-suite" \
    "ros-${ROS_DISTRO}-teleop-twist-keyboard" \
    "ros-${ROS_DISTRO}-xacro" \
    "ros-${ROS_DISTRO}-tf2-tools" \
    "ros-${ROS_DISTRO}-robot-localization" \
    python3-colcon-common-extensions \
    python3-colcon-ros \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    build-essential \
    git
success "ROS 2 ${ROS_DISTRO} installed"

# rosdep — first-run init is idempotent-safe (warn and continue if already initialized)
info "Initializing rosdep..."
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update || warn "rosdep update returned non-zero — continuing"
success "rosdep ready"

# ── Step 4: Miniconda ─────────────────────────────────────────────────────────
step "Step 4 — Miniconda"

if [ -d "$MINICONDA_DIR" ] || command -v conda &>/dev/null; then
    success "Miniconda already installed — skipping"
else
    info "Downloading Miniconda for $(uname -m)..."
    curl -fsSL "$MINICONDA_URL" -o "$MINICONDA_INSTALLER"
    bash "$MINICONDA_INSTALLER" -b -p "$MINICONDA_DIR"
    rm -f "$MINICONDA_INSTALLER"
    success "Miniconda installed at $MINICONDA_DIR"
fi

# Make conda available in this shell session
export PATH="$MINICONDA_DIR/bin:$PATH"
# shellcheck disable=SC1091
source "$MINICONDA_DIR/etc/profile.d/conda.sh"

# ── Step 5: Create ros2_jazzy conda environment ───────────────────────────────
step "Step 5 — conda env: $CONDA_ENV"

# Pin python=3.12 so apt-ROS rclpy .so files (built against system python3.12)
# load cleanly inside this env. Don't change this without checking ABI compat.
if conda env list | grep -qE "^${CONDA_ENV}\s"; then
    success "Environment '$CONDA_ENV' already exists — skipping creation"
else
    info "Creating conda environment '$CONDA_ENV' with python=3.12..."
    # --override-channels + conda-forge avoids the defaults-channel TOS gate
    # introduced in newer conda releases (CondaToSNonInteractiveError).
    conda create -n "$CONDA_ENV" -y --quiet \
        --override-channels --channel conda-forge \
        python=3.12 pip
    success "Environment '$CONDA_ENV' created"
fi

# ── Step 6: Python extras (pyproject.toml deps) ───────────────────────────────
step "Step 6 — Python extras"

info "Activating conda env '$CONDA_ENV'..."
conda activate "$CONDA_ENV"

info "Installing project dependencies from pyproject.toml (editable mode)..."
pip install --upgrade pip --quiet
pip install -e "$PROJECT_DIR" --quiet
success "Python extras installed"

# ── Step 7: Build workspace ───────────────────────────────────────────────────
step "Step 7 — colcon build"

info "Cleaning previous build artifacts..."
rm -rf "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"

info "Building the workspace..."
# Apt-ROS Python deps (catkin_pkg, empy, lark, ament_package, ...) live in
# /usr/lib/python3/dist-packages, which conda's python doesn't see by default.
# Append it so colcon/ament_cmake can import them at build time.
bash -c "
    set -e
    source \"$MINICONDA_DIR/etc/profile.d/conda.sh\"
    conda activate \"$CONDA_ENV\"
    source /opt/ros/${ROS_DISTRO}/setup.bash
    export PYTHONPATH=\"\${PYTHONPATH}:/usr/lib/python3/dist-packages\"
    cd \"${WORKSPACE}\"
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_cohesion+
"
success "Workspace built successfully"

# ── Step 8: Configure ~/.bashrc ───────────────────────────────────────────────
step "Step 8 — Shell config (~/.bashrc)"

BASHRC="$HOME/.bashrc"
MARKER="# >>> ros2_jazzy workspace setup >>>"
MARKER_END="# <<< ros2_jazzy workspace setup <<<"

if grep -q "$MARKER" "$BASHRC" 2>/dev/null; then
    success ".bashrc already configured — skipping"
else
    info "Adding ROS 2 environment setup to ~/.bashrc..."
    cat >> "$BASHRC" << EOF

$MARKER
# Added by setup_ubuntu.sh — Differential Drive Mobile Robot
source ${MINICONDA_DIR}/etc/profile.d/conda.sh
conda activate ${CONDA_ENV}
source /opt/ros/${ROS_DISTRO}/setup.bash
export PYTHONPATH="\${PYTHONPATH}:/usr/lib/python3/dist-packages"
[ -f ${WORKSPACE}/install/setup.bash ] && source ${WORKSPACE}/install/setup.bash
$MARKER_END
EOF
    success ".bashrc updated"
fi

# ── Done ──────────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}${BOLD}╔══════════════════════════════════════════════════════╗${RESET}"
echo -e "${GREEN}${BOLD}║        Setup complete!                               ║${RESET}"
echo -e "${GREEN}${BOLD}╚══════════════════════════════════════════════════════╝${RESET}"
echo ""
echo -e "  Workspace : ${BOLD}${WORKSPACE}${RESET}"
echo -e "  ROS distro: ${BOLD}/opt/ros/${ROS_DISTRO}${RESET}"
echo -e "  Env       : ${BOLD}conda activate ${CONDA_ENV}${RESET}"
echo -e "  Build dir : ${BOLD}${WORKSPACE}/install${RESET}"
echo ""
echo -e "  ${YELLOW}Next step:${RESET} Open a new terminal (or run ${BOLD}source ~/.bashrc${RESET})"
echo -e "  Then test with: ${BOLD}ros2 topic list${RESET}"
echo ""
