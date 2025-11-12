#!/usr/bin/env bash
set -euo pipefail

# ======= 사용자 조절 옵션(인자 없이 여기만 바꾸면 됨) =======
DROP_CACHES=1          # 1: 시작 전에 page cache drop( sudo 필요 ), 0: 사용 안 함
USE_PERF=0             # 1: perf stat 켜기( sudo 권장 ), 0: 끔
PERF_EVENTS="cycles,instructions,cache-misses,context-switches,cpu-migrations,page-faults"
PIN_RECORDER_CPU=""    # 예: "0" 로스백 녹화 프로세스를 코어 0에 핀닝. 비우면 사용 안 함.
# ============================================================

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <runtime_in_seconds>" >&2
  exit 1
fi

RUNTIME="$1"
TS="$(date +%F_%H-%M-%S)"
OUTDIR="rosbag_benchmark_${TS}"
mkdir -p "$OUTDIR"
echo "[record] run=${RUNTIME}s  outdir=${OUTDIR}"

# 종속 툴 체크(있으면 사용, 없어도 진행)
have_cmd() { command -v "$1" >/dev/null 2>&1; }

# 종료/중단 시 정리
PIDS=()
cleanup() {
  echo
  echo "[record] cleaning up..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  echo "[record] done. outputs in: ${OUTDIR}"
}
trap cleanup EXIT INT TERM

# (선택) 캐시 드랍
if [[ "${DROP_CACHES}" -eq 1 ]]; then
  if [[ $EUID -ne 0 ]]; then
    echo "[record] dropping caches (sudo)..."
    sudo bash -c 'echo 3 > /proc/sys/vm/drop_caches' || true
  else
    echo 3 > /proc/sys/vm/drop_caches || true
  fi
fi

# 환경 스냅샷
{
  echo "===== uname -a ====="; uname -a
  echo "===== lscpu ====="; have_cmd lscpu && lscpu || true
  echo "===== lsblk ====="; have_cmd lsblk && lsblk -o NAME,TYPE,SIZE,ROTA,MOUNTPOINT || true
  echo "===== df -h ====="; df -h
  echo "===== free -m ====="; free -m
} > "${OUTDIR}/env_snapshot.txt" 2>&1 || true

# rosbag 레코더 시작 (-a: 모든 토픽)
REC_LOG="${OUTDIR}/rosbag_record.log"
REC_DIR="${OUTDIR}/bag"

if [[ -n "${PIN_RECORDER_CPU}" ]]; then
  echo "[record] starting rosbag recorder (pinned to cpu ${PIN_RECORDER_CPU})..."
  taskset -c "${PIN_RECORDER_CPU}" ros2 bag record -a -o "${REC_DIR}" > "${REC_LOG}" 2>&1 &
else
  echo "[record] starting rosbag recorder..."
  ros2 bag record -a -o "${REC_DIR}" > "${REC_LOG}" 2>&1 &
fi
REC_PID=$!
PIDS+=("${REC_PID}")
echo "[record] recorder PID=${REC_PID}"

# 모니터들 시작 (가능한 것만)
echo "[record] starting monitors..."
if have_cmd mpstat; then
  mpstat -P ALL 1 "${RUNTIME}" > "${OUTDIR}/mpstat_cpu.log" 2>&1 &
  PIDS+=($!)
fi
if have_cmd iostat; then
  iostat -d -x -k 1 "${RUNTIME}" > "${OUTDIR}/iostat_disk.log" 2>&1 &
  PIDS+=($!)
fi
if have_cmd vmstat; then
  vmstat 1 "${RUNTIME}" > "${OUTDIR}/vmstat_memory.log" 2>&1 &
  PIDS+=($!)
fi
if have_cmd pidstat; then
  # rosbag 레코더 개별 프로세스의 CPU/메모리/IO 추적
  pidstat -dur -h -p "${REC_PID}" 1 "${RUNTIME}" > "${OUTDIR}/pidstat_recorder.log" 2>&1 &
  PIDS+=($!)
fi
if [[ "${USE_PERF}" -eq 1 ]] && have_cmd perf; then
  # system-wide perf (1초 간격 리포트)
  if [[ $EUID -ne 0 ]]; then
    echo "[record] starting perf (sudo) ..."
    sudo perf stat -a -I 1000 -e "${PERF_EVENTS}" -- sleep "${RUNTIME}" > "${OUTDIR}/perf_stat.log" 2>&1 &
  else
    perf stat -a -I 1000 -e "${PERF_EVENTS}" -- sleep "${RUNTIME}" > "${OUTDIR}/perf_stat.log" 2>&1 &
  fi
  PIDS+=($!)
fi

# 지정 시간만큼 대기
echo "[record] running for ${RUNTIME}s ..."
sleep "${RUNTIME}"

# rosbag 정상 종료(SIGINT) → 메타/인덱스 flush
echo "[record] stopping recorder (SIGINT) ..."
kill -2 "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" 2>/dev/null || true

# dmesg tail 저장(에러 확인용)
have_cmd dmesg && dmesg | tail -n 200 > "${OUTDIR}/dmesg_tail.txt" 2>&1 || true

echo "[record] finished. bag dir: ${REC_DIR}"
