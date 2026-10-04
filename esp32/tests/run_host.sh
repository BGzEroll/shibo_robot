#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
shibo_test_dir=$(mktemp -d)
trap 'rm -rf "$shibo_test_dir"' EXIT
shibo_sources="user_lib/controller/action.cpp user_lib/controller/balance.cpp user_lib/controller/input.cpp"
shibo_includes="-Iuser_lib -Imanaged_components/espressif__cjson/cJSON -Itests/stubs"
# 直接提取生产代码中的解析函数，主机测试无需编译 BLE 硬件部分。
python3 - "$shibo_test_dir/report.cpp" <<'PYTHON'
from pathlib import Path
import sys

source = Path("user_lib/hw/gamepad.cpp").read_text()
start = source.index("    bool parse_report(")
body = source.index("{", start)
depth = 0
for end in range(body, len(source)):
    depth += (source[end] == "{") - (source[end] == "}")
    if depth == 0:
        break
Path(sys.argv[1]).write_text(
    '#include "hw/gamepad.h"\n\nnamespace gamepad\n{\n' + source[start:end + 1] + '\n}\n')
PYTHON
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/control_test.cpp \
    $shibo_sources "$shibo_test_dir/report.cpp" -o "$shibo_test_dir/control_test"
"$shibo_test_dir/control_test"
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/safety_test.cpp \
    $shibo_sources user_lib/controller/control.cpp -o "$shibo_test_dir/safety_test"
"$shibo_test_dir/safety_test"
gcc -std=c11 -Imanaged_components/espressif__cjson/cJSON -c \
    managed_components/espressif__cjson/cJSON/cJSON.c -o "$shibo_test_dir/cjson.o"
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/config_test.cpp \
    user_lib/config.cpp "$shibo_test_dir/cjson.o" -o "$shibo_test_dir/config_test"
"$shibo_test_dir/config_test"
