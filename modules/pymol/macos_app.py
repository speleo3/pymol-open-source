"""
Install the PyMOL app launcher on macOS.

Wheels for macOS include a launcher app bundle (pymol/macos/*.app). pip
cannot install it to the Applications folder, so this module copies it there
and connects it to the current Python environment.

Usage:
    python -m pymol.macos_app install [--dest DIR]
    python -m pymol.macos_app uninstall [--dest DIR]
"""

import argparse
import plistlib
import shutil
import subprocess
import sys
from pathlib import Path

LSREGISTER = (
    "/System/Library/Frameworks/CoreServices.framework/Frameworks/"
    "LaunchServices.framework/Support/lsregister"
)

DEFAULT_DEST = Path.home() / "Applications"


def bundled_app() -> Path:
    """Return the app bundle shipped inside the pymol package."""
    apps = sorted((Path(__file__).parent / "macos").glob("*.app"))
    if not apps:
        raise SystemExit(
            "Error: This PyMOL installation does not include the macOS app "
            "launcher. Only the macOS wheels include it."
        )
    return apps[0]


def bundle_identifier(app: Path):
    try:
        with open(app / "Contents" / "Info.plist", "rb") as handle:
            return plistlib.load(handle).get("CFBundleIdentifier")
    except (OSError, plistlib.InvalidFileException):
        return None


def _check_replaceable(target: Path, identifier: str) -> None:
    """Refuse to touch an existing app which is not a PyMOL launcher."""
    if bundle_identifier(target) != identifier:
        raise SystemExit(
            f"Error: {target} exists and is not a PyMOL app launcher. "
            "Remove it yourself or use --dest."
        )


def _lsregister(*args: str) -> None:
    if Path(LSREGISTER).exists():
        subprocess.run([LSREGISTER, *args], check=False)


def install(dest: Path) -> Path:
    src = bundled_app()
    identifier = bundle_identifier(src)
    target = dest / src.name

    if target.exists() or target.is_symlink():
        _check_replaceable(target, identifier)
        shutil.rmtree(target)

    dest.mkdir(parents=True, exist_ok=True)
    shutil.copytree(src, target, symlinks=True)

    # The launcher runs <app>/Contents/venv/bin/python. Python detects a
    # venv from pyvenv.cfg next to the unresolved path of the executable,
    # so for a venv, link the whole venv folder. Otherwise, link the
    # interpreter (Python resolves that symlink to find its prefix).
    venv_link = target / "Contents" / "venv"
    prefix = Path(sys.prefix).absolute()
    if sys.prefix != sys.base_prefix and (prefix / "bin" / "python").exists():
        venv_link.symlink_to(prefix)
    else:
        bindir = venv_link / "bin"
        bindir.mkdir(parents=True)
        (bindir / "python").symlink_to(Path(sys.executable).absolute())

    _lsregister("-f", str(target))
    return target


def uninstall(dest: Path) -> Path:
    target = dest / bundled_app().name
    if not (target.exists() or target.is_symlink()):
        raise SystemExit(f"Error: {target} does not exist")
    _check_replaceable(target, bundle_identifier(bundled_app()))
    _lsregister("-u", str(target))
    shutil.rmtree(target)
    return target


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        prog="python -m pymol.macos_app",
        description="Install the PyMOL app launcher on macOS.",
    )
    parser.add_argument("action", choices=("install", "uninstall"))
    parser.add_argument(
        "--dest",
        type=Path,
        default=DEFAULT_DEST,
        help=f"folder for the app (default: {DEFAULT_DEST})",
    )
    args = parser.parse_args(argv)

    if sys.platform != "darwin":
        raise SystemExit("Error: The app launcher is only for macOS.")

    dest = args.dest.expanduser().absolute()
    if args.action == "install":
        target = install(dest)
        print(f"Installed {target}")
        print(f"It runs PyMOL with {sys.executable}")
    else:
        target = uninstall(dest)
        print(f"Removed {target}")


if __name__ == "__main__":
    main()
