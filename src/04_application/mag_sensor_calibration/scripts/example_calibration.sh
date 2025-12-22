#!/bin/bash
# 传感器校正使用示例脚本

# 设置默认参数
ARM="arm1"
START_X=0.5
START_Y=0.0
START_Z=0.3
ROTATION_CENTER_X=0.5
ROTATION_CENTER_Y=0.0
ROTATION_CENTER_Z=0.3
ROTATION_AXIS="z"
ANGLE_MIN=0.0
ANGLE_MAX=3.14159
ANGLE_STEP=0.174533  # 约10度
VELOCITY_SCALING=0.2
COLLECTION_DURATION=2.0
OUTPUT_DIR="/tmp/calibration_data"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --arm)
            ARM="$2"
            shift 2
            ;;
        --start-x)
            START_X="$2"
            shift 2
            ;;
        --start-y)
            START_Y="$2"
            shift 2
            ;;
        --start-z)
            START_Z="$2"
            shift 2
            ;;
        --angle-min)
            ANGLE_MIN="$2"
            shift 2
            ;;
        --angle-max)
            ANGLE_MAX="$2"
            shift 2
            ;;
        --angle-step)
            ANGLE_STEP="$2"
            shift 2
            ;;
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            exit 1
            ;;
    esac
done

echo "启动传感器校正..."
echo "参数:"
echo "  机械臂: $ARM"
echo "  起始位置: ($START_X, $START_Y, $START_Z)"
echo "  旋转中心: ($ROTATION_CENTER_X, $ROTATION_CENTER_Y, $ROTATION_CENTER_Z)"
echo "  旋转轴: $ROTATION_AXIS"
echo "  角度范围: [$ANGLE_MIN, $ANGLE_MAX] rad"
echo "  角度步长: $ANGLE_STEP rad"
echo "  输出目录: $OUTPUT_DIR"
echo ""

# 创建输出目录
mkdir -p "$OUTPUT_DIR"

# 启动校正
SESSION_ID=$(rosservice call /calibration_controller/start_calibration \
  "arm: '$ARM'
   start_pose:
     position: {x: $START_X, y: $START_Y, z: $START_Z}
     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
   rotation_center: {x: $ROTATION_CENTER_X, y: $ROTATION_CENTER_Y, z: $ROTATION_CENTER_Z}
   rotation_axis: '$ROTATION_AXIS'
   angle_range: [$ANGLE_MIN, $ANGLE_MAX]
   angle_step: $ANGLE_STEP
   velocity_scaling: $VELOCITY_SCALING
   data_collection_duration: $COLLECTION_DURATION
   save_raw_data: true
   output_directory: '$OUTPUT_DIR'" | grep -oP 'session_id: "\K[^"]+')

if [ -z "$SESSION_ID" ]; then
    echo "错误: 无法启动校正"
    exit 1
fi

echo "校正已启动，会话ID: $SESSION_ID"
echo "等待校正完成（按Ctrl+C可提前停止）..."

# 等待用户中断或校正完成
trap 'echo ""; echo "正在停止校正..."; rosservice call /calibration_controller/stop_calibration "session_id: '\''$SESSION_ID'\'' save_data: true" > /dev/null 2>&1; echo "校正已停止"; exit 0' INT

# 监控校正进度
while true; do
    sleep 5
    # 可以添加进度查询逻辑
done

