#!/bin/bash
# 编译并系统安装重定位依赖库: small_gicp
# PC / NX 通用,重装直接重跑。
# 无 TTY 环境(如 ssh 远程执行)可用: SUDO_PASS=密码 bash install_deps.sh
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"

do_sudo() {
  if [ -n "$SUDO_PASS" ]; then
    echo "$SUDO_PASS" | sudo -S "$@"
  else
    sudo "$@"
  fi
}

export LC_ALL=C

echo "== 1/1 small_gicp =="
cd "$DIR/small_gicp"
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
cmake --build build -j"$(nproc)"
do_sudo cmake --install build
echo "== 全部安装完成 =="
