"""Runtime xacro → URDF loader (no ROS 2 required).

Usage:
    from xacro_loader import urdf_from_xacro

    with urdf_from_xacro(xacro_path, pkg_map) as urdf_path:
        robot = scene.add_entity(gs.morphs.URDF(file=urdf_path))
"""
import os
import re
import subprocess
import tempfile
from contextlib import contextmanager
from pathlib import Path


@contextmanager
def urdf_from_xacro(xacro_path: Path | str, pkg_map: dict[str, Path | str]):
    """Process a xacro file and yield a temporary plain-URDF path.

    Args:
        xacro_path: path to the .urdf.xacro source file
        pkg_map:    mapping of ROS package name → local directory path,
                    used to resolve $(find <pkg>) substitutions
    """
    content = Path(xacro_path).read_text()

    for pkg_name, pkg_path in pkg_map.items():
        content = content.replace(f"$(find {pkg_name})", str(pkg_path))

    # Strip file:// URIs so non-ROS loaders can find meshes
    content = re.sub(r'filename="file://', 'filename="', content)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf.xacro", delete=False) as src_tmp:
        src_tmp.write(content)
        src_tmp_path = src_tmp.name

    try:
        result = subprocess.run(
            ["xacro", src_tmp_path], capture_output=True, text=True, check=True
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"xacro failed:\n{e.stderr}") from e
    finally:
        os.unlink(src_tmp_path)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as urdf_tmp:
        urdf_tmp.write(result.stdout)
        urdf_tmp_path = urdf_tmp.name

    try:
        yield urdf_tmp_path
    finally:
        os.unlink(urdf_tmp_path)
