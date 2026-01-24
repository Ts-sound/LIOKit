# LIOKit

LIO-SAM engine, packaged as a cross-platform SDK for Linux and Windows.

vcpkg export --output-dir=. --output=vcpkg_export --raw

cmake -B /opt/tong/ws/git_repo/LIOKit/src/LIOKit/build -S /opt/tong/ws/git_repo/LIOKit/src/LIOKit \
  -DCMAKE_TOOLCHAIN_FILE=/opt/tong/ws/git_repo/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_INSTALLED_DIR=/opt/tong/ws/git_repo/LIOKit/src/LIOKit/vcpkg_installed

cmake -B /opt/tong/ws/git_repo/LIOKit/src/LIOKit/build/build_release -S /opt/tong/ws/git_repo/LIOKit/src/LIOKit -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=/opt/tong/ws/git_repo/LIOKit/../vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_INSTALLED_DIR=/opt/tong/ws/git_repo/LIOKit/src/LIOKit/vcpkg_installed
