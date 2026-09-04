"""
Builds the native extensions.
"""

import argparse
import pathlib
import shutil
import subprocess
import sys
from subprocess import check_call


def _run(cmd: list[str | pathlib.Path]):
    """Run ``cmd``, surfacing the tail of its output when it fails.

    pip/poetry hide the build script's output on failure, so Meson errors
    were impossible to diagnose from ``pip install``. Printing the tail
    (stderr first, stdout as fallback) fixes that.
    """
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        tail = "\n".join((result.stderr or "").splitlines()[-40:])
        if not tail:
            tail = "\n".join((result.stdout or "").splitlines()[-40:])
        print(f"Command failed ({result.returncode}): {' '.join(map(str, cmd))}", file=sys.stderr)
        print(tail, file=sys.stderr)
        raise subprocess.CalledProcessError(result.returncode, cmd)


def parse_args():
    parser = argparse.ArgumentParser(prog="build_extensions")

    parser.add_argument(
        "--build_dir",
        default="build",
        help="Directory for Meson to use while building extensions.",
    )
    parser.add_argument(
        "--build_type",
        default="release",
        choices=["debug", "debugoptimized", "release"],
        help="The type of build to provide. Defaults to release mode.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Clean build and installation directories before building.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Whether to print more verbose compilation output.",
    )
    parser.add_argument(
        "--use_pgo",
        action="store_true",
        help="Whether to enable profile-guided optimisation.",
    )
    parser.add_argument(
        "--pgo_workload",
        default="default",
        choices=["default", "break"],
        help=(
            "Which workload to profile when --use_pgo is set. 'default' "
            "runs the historical pytest + VRPLIB workload. 'break' runs "
            "the fork's break-path benchmark instead, since none of the "
            "default instances exercise custom breaks."
        ),
    )
    parser.add_argument(
        "--additional",
        nargs=argparse.REMAINDER,
        default=[],
        help="Extra Meson configuration options (passed verbatim to Meson).",
    )

    return parser.parse_args()


def clean(build_dir: pathlib.Path, install_dir: pathlib.Path):
    # shutil.rmtree is cross-platform; the previous `check_call(["rm", ...])`
    # broke `pip install .` on Windows (rm is Unix-only).
    shutil.rmtree(build_dir, ignore_errors=True)

    for extension in install_dir.rglob("*.so"):
        extension.unlink()

    for extension in install_dir.rglob("*.pyd"):
        extension.unlink()


def configure(
    build_dir: pathlib.Path,
    build_type: str,
    *additional: list[str],
):
    cwd = pathlib.Path.cwd()
    # fmt: off
    args = [
        build_dir,
        "--buildtype", build_type,
        f"-Dpython.platlibdir={cwd.absolute()}",
        f"-Dstrip={'true' if build_type == 'release' else 'false'}",
        f"-Db_coverage={'true' if build_type != 'release' else 'false'}",
        *additional,
    ]
    # fmt: on

    cmd = "configure" if build_dir.exists() else "setup"
    _run(["meson", cmd, *args])


def compile(build_dir: pathlib.Path, verbose: bool):
    args = ["-C", build_dir] + (["--verbose"] if verbose else [])
    _run(["meson", "compile", *args])


def install(build_dir: pathlib.Path):
    _run(["meson", "install", "-C", build_dir, "--skip-subprojects"])


def build(
    build_dir: pathlib.Path,
    build_type: str,
    verbose: bool,
    *additional: list[str],
):
    configure(build_dir, build_type, *additional)
    compile(build_dir, verbose)
    install(build_dir)


def workload():
    # TODO if and when we actually start using PGO we should probably rethink
    # what the profiling workload needs to be. For example, larger instances
    # are harder to solve, so perhaps we should optimise for those?
    cmds = [
        "pytest",
        "pyvrp --seed 1 tests/data/X-n101-50-k13.vrp --max_runtime 5",
        "pyvrp --seed 2 tests/data/RC208.vrp --max_runtime 5",
    ]

    for cmd in cmds:
        check_call(cmd.split())


def main():
    args = parse_args()
    cwd = pathlib.Path.cwd()
    build_dir = cwd / args.build_dir

    if args.clean:
        install_dir = cwd / "pyvrp"
        clean(build_dir, install_dir)

    build_args = (
        build_dir,
        args.build_type,
        args.verbose,
        *args.additional,
    )

    if args.use_pgo:
        build(*build_args, "-Db_pgo=generate")
        workload()
        build(*build_args, "-Db_pgo=use")
    else:
        build(*build_args)


if __name__ == "__main__":
    main()
