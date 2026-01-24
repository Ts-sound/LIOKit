# LIOKit

LIO-SAM engine, packaged as a cross-platform SDK for Linux and Windows.

vcpkg export --output-dir=. --output=vcpkg_export --raw

cmake -B build -S . \
  -DCMAKE_TOOLCHAIN_FILE=/opt/tong/ws/git_repo/LIOKit/src/3rdparty/vcpkg_export/scripts/buildsystems/vcpkg.cmake -DVCPKG_INSTALLED_DIR=/opt/tong/ws/git_repo/LIOKit/src/3rdparty/vcpkg_export/installed
