import argparse, os, glob, logging
from pathlib import Path

logging.basicConfig(level=logging.DEBUG, format="%(levelname)-8s %(asctime)s %(lineno)-4d  %(message)s")

workpath = str(Path(__file__).resolve().parent.parent)

logging.debug(f"workpath: {workpath}")


def run_cmd(cmd):
    logging.info(f"run cmd: {cmd}")
    return os.system(cmd)


def build_liokit(build_type='Release'):
    src_path = workpath + "/src/LIOKit"
    build_dir = f"{src_path}/build/build_{build_type}"
    toolchain_file = workpath + "/../vcpkg/scripts/buildsystems/vcpkg.cmake"
    vcpkg_installed_dir = workpath + "/src/LIOKit/vcpkg_installed"

    run_cmd(f"rm -r {build_dir}")

    # Run cmake configure
    cmake_cmd = f"cmake -B {build_dir} -S {src_path} -DCMAKE_BUILD_TYPE={build_type} -DCMAKE_TOOLCHAIN_FILE={toolchain_file} -DVCPKG_INSTALLED_DIR={vcpkg_installed_dir}"
    run_cmd(cmake_cmd)

    # Run cmake build
    build_cmd = f"cmake --build {build_dir} -- -j{os.cpu_count()}"
    run_cmd(build_cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build LIOKit")
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--debug', action='store_true', help='Build in Debug mode')
    group.add_argument('--release', action='store_true', help='Build in Release mode (default)')

    args = parser.parse_args()

    build_type = 'Release'
    if args.debug:
        build_type = 'Debug'

    build_liokit(build_type)
