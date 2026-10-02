#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
shibo_test_dir=$(mktemp -d)
trap 'rm -rf "$shibo_test_dir"' EXIT
shibo_sources="user_lib/controller/action.cpp user_lib/controller/balance.cpp user_lib/controller/input.cpp"
shibo_includes="-Iuser_lib -Imanaged_components/espressif__cjson/cJSON -Itests/stubs"
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/control_test.cpp \
    $shibo_sources user_lib/hw/gamepad_report.cpp -o "$shibo_test_dir/control_test"
"$shibo_test_dir/control_test"
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/safety_test.cpp \
    $shibo_sources user_lib/controller/control.cpp -o "$shibo_test_dir/safety_test"
"$shibo_test_dir/safety_test"
gcc -std=c11 -Imanaged_components/espressif__cjson/cJSON -c \
    managed_components/espressif__cjson/cJSON/cJSON.c -o "$shibo_test_dir/cjson.o"
g++ -std=c++17 -Wall -Wextra -Werror $shibo_includes tests/config_test.cpp \
    user_lib/config.cpp "$shibo_test_dir/cjson.o" -o "$shibo_test_dir/config_test"
"$shibo_test_dir/config_test"
