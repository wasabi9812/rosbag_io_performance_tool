#!/bin/bash

# --- 설정 ---
# 1. 실행 시간을 스크립트의 첫 번째 인자로 받음
if [ $# -lt 1 ]; then
  echo "Usage: $0 <runtime_in_seconds>" >&2
  echo "Example: ./rosbag_benchmark.sh 30"
  exit 1
fi

RUNTIME=$1
LOG_DIR="rosbag_benchmark_$(date +%F_%H-%M-%S)"
mkdir -p "$LOG_DIR"
echo "Starting benchmark for $RUNTIME seconds..."
echo "All logs will be saved in: $LOG_DIR"

# --- 종료 시 모든 백그라운드 프로세스를 정리하는 함수 ---
# (Ctrl+C를 누르거나 스크립트가 끝나면 실행됨)
pids_to_kill=()
cleanup() {
  echo
  echo "Benchmark time ($RUNTIME sec) finished. Cleaning up all background processes..."
  for pid in "${pids_to_kill[@]}"; do
    kill "$pid" 2>/dev/null
  done
  echo "Cleanup done."
}
trap cleanup EXIT INT TERM

# --- 1. 캐시 비우기 (fio.sh와 동일) ---
echo "Dropping OS page caches (requires sudo)..."
sudo bash -c "echo 3 > /proc/sys/vm/drop_caches"
echo "Caches dropped."

# --- 2. ROS 부하 발생기 (Publisher) 시작 ---
# (fio의 --numjobs=12와 유사)
echo "Starting ROS Publisher (multi_robot_image_launch.py) in background..."
ros2 launch robot_image_publisher multi_robot_image_launch.py &
LAUNCH_PID=$!
pids_to_kill+=($LAUNCH_PID)

# --- 3. ROS 테스트 대상 (Recorder) 시작 ---
# (fio의 --filename=/dev/nvme0n1와 유사)
echo "Starting ROS Recorder (ros2 bag record) in background..."
ros2 bag record -a -o "$LOG_DIR/my_bag" &
RECORD_PID=$!
pids_to_kill+=($RECORD_PID)

# ROS 노드가 뜰 때까지 3초 대기
sleep 3

# --- 4. 성능 계측기 (Monitoring) 시작 ---
echo "Starting all monitors (mpstat, iostat, vmstat)..."

# mpstat (CPU 사용률): Lab 2에서 사용
# -P ALL (모든 코어), 1 (1초 간격)
mpstat -P ALL 1 $RUNTIME > "$LOG_DIR/mpstat_cpu.log" &
pids_to_kill+=($!)

# iostat (I/O 사용률): 사용자가 요청한 'iostat'
# -d (디스크만), -x (상세 통계), -k (KB 단위), 1 (1초 간격)
iostat -d -x -k 1 $RUNTIME > "$LOG_DIR/iostat_disk.log" &
pids_to_kill+=($!)

# vmstat (메모리 사용률): 사용자가 요청한 'mem'
# 1 (1초 간격)
vmstat 1 $RUNTIME > "$LOG_DIR/vmstat_memory.log" &
pids_to_kill+=($!)

# --- 5. 벤치마크 실행 대기 (fio의 --runtime) ---
echo "Benchmark running... (Publisher: $LAUNCH_PID, Recorder: $RECORD_PID)"
echo "Monitoring PIDs: ${pids_to_kill[@]:2}"
echo "Will stop automatically after $RUNTIME seconds. (Press Ctrl+C to stop early)"

# 이 sleep이 메인 타이머입니다.
sleep $RUNTIME

# --- 6. 종료 ---
# (스크립트가 여기서 끝나면 'trap'이 cleanup 함수를 자동 호출)