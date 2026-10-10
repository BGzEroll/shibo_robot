#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
shibo_test_dir=$(mktemp -d)
trap 'rm -rf "$shibo_test_dir"' EXIT
shibo_sources="user_lib/controller/action.cpp user_lib/controller/balance.cpp"
shibo_includes="-Iuser_lib -Imanaged_components/espressif__cjson/cJSON -Itests/stubs"
# 提取生产代码中的报告解析、输入路由和电压换算，主机测试无需编译硬件驱动。
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

source = Path("user_lib/controller/input.cpp").read_text()
headers = source[:source.index("namespace host")]
headers = headers.replace('#include "input.h"', '#include "controller/input.h"')
headers = headers.replace('#include "action.h"', '#include "controller/action.h"')
headers = '\n'.join(line for line in headers.splitlines()
    if not line.startswith(('#include "driver/', '#include "freertos/')))
Path(sys.argv[1]).with_name("input.cpp").write_text(
    headers + '\n\n' + source[source.index("namespace control::input_router"):])

source = Path("user_lib/hw/motor.cpp").read_text()
constants = source[source.index("        constexpr int32_t Q15_ONE"):source.index("        struct context")]
start = source.index("        bool update_bus_voltage(")
end = source.index("\n        /**", start)
Path(sys.argv[1]).with_name("motor_voltage.cpp").write_text(
    '#include "hw/motor.h"\n#include "hw/battery.h"\n\nnamespace motor\n{\n' +
    constants + source[start:end] + '\n}\n')
PYTHON
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/control_test.cpp \
    $shibo_sources "$shibo_test_dir/input.cpp" "$shibo_test_dir/report.cpp" -o "$shibo_test_dir/control_test"
"$shibo_test_dir/control_test"
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/safety_test.cpp \
    $shibo_sources "$shibo_test_dir/input.cpp" user_lib/controller/control.cpp -o "$shibo_test_dir/safety_test"
"$shibo_test_dir/safety_test"
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/leg_test.cpp \
    user_lib/controller/leg.cpp -o "$shibo_test_dir/leg_test"
"$shibo_test_dir/leg_test"
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes -I"$shibo_test_dir" \
    tests/motor_test.cpp -o "$shibo_test_dir/motor_test"
"$shibo_test_dir/motor_test"
gcc -std=c11 -Imanaged_components/espressif__cjson/cJSON -c \
    managed_components/espressif__cjson/cJSON/cJSON.c -o "$shibo_test_dir/cjson.o"
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/config_test.cpp \
    user_lib/config.cpp "$shibo_test_dir/cjson.o" -o "$shibo_test_dir/config_test"
"$shibo_test_dir/config_test"
