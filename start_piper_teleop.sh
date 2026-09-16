#!/usr/bin/env bash

set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
D1_NAMESPACE_SETUP="/opt/d1_ros2/namespace.sh"
TELEOP_SCRIPT="$SCRIPT_DIR/piper_control/example/teleop/betafpv_piper_teleop.py"
CAN_CHANNEL="can1"
CAN_BITRATE="1000000"

can_is_ready() {
    local flags
    if [[ ! -r "/sys/class/net/$CAN_CHANNEL/flags" ]]; then
        return 1
    fi
    read -r flags < "/sys/class/net/$CAN_CHANNEL/flags"
    if (( (flags & 1) == 0 )); then
        return 1
    fi
    ip -details link show "$CAN_CHANNEL" | grep -q "bitrate $CAN_BITRATE"
}

DRY_RUN=false
for arg in "$@"; do
    if [[ "$arg" == "--dry-run" ]]; then
        DRY_RUN=true
        break
    fi
done

if [[ ! -r "$ROS_SETUP" ]]; then
    echo "错误: 找不到 ROS 2 环境文件: $ROS_SETUP" >&2
    exit 1
fi
if [[ ! -r "$D1_NAMESPACE_SETUP" ]]; then
    echo "错误: 找不到 D1 namespace 文件: $D1_NAMESPACE_SETUP" >&2
    exit 1
fi
if [[ ! -f "$TELEOP_SCRIPT" ]]; then
    echo "错误: 找不到遥控程序: $TELEOP_SCRIPT" >&2
    exit 1
fi

if ! python3 -c 'import piper_sdk' >/dev/null 2>&1; then
    echo "未检测到 piper_sdk，正在执行: pip3 install piper_sdk"
    if ! command -v pip3 >/dev/null 2>&1; then
        echo "错误: 找不到 pip3，请先安装 python3-pip" >&2
        exit 1
    fi
    pip3 install piper_sdk
fi

if ! python3 -c 'import piper_sdk' >/dev/null 2>&1; then
    echo "错误: piper_sdk 安装后仍无法导入" >&2
    exit 1
fi

if [[ "$DRY_RUN" == false ]]; then
    if ! ip link show "$CAN_CHANNEL" >/dev/null 2>&1; then
        echo "错误: 找不到 Piper CAN 接口: $CAN_CHANNEL" >&2
        exit 1
    fi

    if ! can_is_ready; then
        echo "检测到 $CAN_CHANNEL 未启动或波特率不正确，正在配置为 ${CAN_BITRATE}bps..."
        if (( EUID == 0 )); then
            ip link set "$CAN_CHANNEL" down
            ip link set "$CAN_CHANNEL" type can bitrate "$CAN_BITRATE"
            ip link set "$CAN_CHANNEL" up
        else
            if [[ ! -t 0 ]]; then
                echo "错误: 非交互服务无法使用 sudo，请重新安装 d1_piper_joy.service" >&2
                exit 1
            fi
            echo "需要输入 D1 用户的 sudo 密码；密码不会被保存。"
            sudo -v
            sudo ip link set "$CAN_CHANNEL" down
            sudo ip link set "$CAN_CHANNEL" type can bitrate "$CAN_BITRATE"
            sudo ip link set "$CAN_CHANNEL" up
        fi
    fi

    if ! can_is_ready; then
        echo "错误: $CAN_CHANNEL 启动失败" >&2
        exit 1
    fi
    echo "$CAN_CHANNEL 已启动，波特率 ${CAN_BITRATE}bps"
fi

# shellcheck disable=SC1091
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$D1_NAMESPACE_SETUP"

if [[ -z "${ROBOT_NS:-}" ]]; then
    echo "错误: namespace.sh 未设置 ROBOT_NS" >&2
    exit 1
fi

echo "启动 Piper 遥控: joy=/${ROBOT_NS}/joy, can=$CAN_CHANNEL"
echo "只有 use_sdk=true 时才允许控制；Ctrl+C 退出"

exec python3 "$TELEOP_SCRIPT" \
    --joy-topic "/${ROBOT_NS}/joy" \
    --input-format raw \
    --can "$CAN_CHANNEL" \
    "$@"
