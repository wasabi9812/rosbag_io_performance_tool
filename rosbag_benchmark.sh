#!/usr/bin/env bash
set -euo pipefail

# ======= 사용자 옵션 =======
DROP_CACHES=1
USE_PERF=0
PERF_EVENTS="cycles,instructions,cache-misses,context-switches,cpu-migrations,page-faults"
PIN_RECORDER_CPU=""     # 예: "0"
BAG_ROOT="/mnt/nvme_remote"   # rosbag 데이터(대상 I/O) 저장 위치 = NVMe-TCP 마운트
LOG_ROOT="."                  # 모니터링 로그 저장 위치 = 로컬(현재 디렉터리)
# ==========================

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <runtime_in_seconds>" >&2
  exit 1
fi

RUNTIME="$1"
TS="$(date +%F_%H-%M-%S)"

# --- 경로 구성: 로그는 로컬, rosbag은 원격(NVMe) ---
OUTDIR="${LOG_ROOT%/}/rosbag_benchmark_${TS}"   # 로그 디렉터리(로컬)
REC_DIR="${BAG_ROOT%/}/rosbag_${TS}"            # rosbag 디렉터리(원격)

mkdir -p "${OUTDIR}"
echo "[record] run=${RUNTIME}s"
echo "[record] LOG_DIR = ${OUTDIR}   (local)"
echo "[record] BAG_DIR = ${REC_DIR}  (NVMe remote)"

# --- 의존 명령 체크 ---
have_cmd() { command -v "$1" >/dev/null 2>&1; }

# --- 종료/중단 시 정리 ---
PIDS=()
cleanup() {
  echo
  echo "[record] cleaning up..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  echo "[record] done."
  echo "[record] logs: ${OUTDIR}"
  echo "[record] bag : ${REC_DIR}"
}
trap cleanup EXIT INT TERM

# --- NVMe 마운트/쓰기 체크 ---
if ! mountpoint -q "${BAG_ROOT}"; then
  echo "[error] ${BAG_ROOT} is not mounted. Mount NVMe target first." >&2
  exit 1
fi
mkdir -p "${REC_DIR}"
# 간단 쓰기 검사
touch "${REC_DIR}/.write_test" 2>/dev/null || { echo "[error] cannot write to ${REC_DIR}"; exit 1; }
rm -f "${REC_DIR}/.write_test"

# --- (선택) 캐시 드랍 ---
if [[ "${DROP_CACHES}" -eq 1 ]]; then
  if [[ $EUID -ne 0 ]]; then
    echo "[record] dropping caches (sudo)..."
    sudo bash -c 'echo 3 > /proc/sys/vm/drop_caches' || true
  else
    echo 3 > /proc/sys/vm/drop_caches || true
  fi
fi

# --- 환경 스냅샷(로그=로컬) ---
{
  echo "===== date ====="; date -R
  echo "===== uname -a ====="; uname -a
  echo "===== lscpu ====="; have_cmd lscpu && lscpu || true
  echo "===== lsblk ====="; have_cmd lsblk && lsblk -o NAME,TYPE,SIZE,ROTA,MOUNTPOINT || true
  echo "===== df -h ====="; df -h
  echo "===== free -m ====="; free -m
  # 스케줄러 상태(가능하면 nvme 블록만)
  for b in /sys/block/nvme*n*/queue/scheduler 2>/dev/null; do
    [[ -e "$b" ]] && echo "===== ${b} =====" && cat "$b"
  done
  echo "===== mount | grep nvme ====="; mount | grep -i nvme || true
} > "${OUTDIR}/env_snapshot.txt" 2>&1 || true

# --- rosbag 레코더 시작 (데이터=원격) ---
REC_LOG="${OUTDIR}/rosbag_record.log"   # rosbag stdout/stderr은 로컬 로그에 남김
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

# 원격 bag 위치를 로컬 로그 폴더에서 쉽게 열도록 심볼릭 링크 추가(옵션)
ln -sfn "${REC_DIR}" "${OUTDIR}/bag_link"

# --- 모니터(모두 로컬에 기록) ---
echo "[record] starting monitors..."
if have_cmd mpstat; then
  mpstat -P ALL 1 "${RUNTIME}" > "${OUTDIR}/mpstat_cpu.log" 2>&1 & PIDS+=($!)
fi
if have_cmd iostat; then
  iostat -d -x -k 1 "${RUNTIME}" > "${OUTDIR}/iostat_disk.log" 2>&1 & PIDS+=($!)
fi
if have_cmd vmstat; then
  vmstat 1 "${RUNTIME}" > "${OUTDIR}/vmstat_memory.log" 2>&1 & PIDS+=($!)
fi
if have_cmd pidstat; then
  pidstat -dur -h -p "${REC_PID}" 1 "${RUNTIME}" > "${OUTDIR}/pidstat_recorder.log" 2>&1 & PIDS+=($!)
fi
if [[ "${USE_PERF}" -eq 1 ]] && have_cmd perf; then
  if [[ $EUID -ne 0 ]]; then
    echo "[record] starting perf (sudo) ..."
    sudo perf stat -a -I 1000 -e "${PERF_EVENTS}" -- sleep "${RUNTIME}" > "${OUTDIR}/perf_stat.log" 2>&1 & PIDS+=($!)
  else
    perf stat -a -I 1000 -e "${PERF_EVENTS}" -- sleep "${RUNTIME}" > "${OUTDIR}/perf_stat.log" 2>&1 & PIDS+=($!)
  fi
fi

# --- 실행 ---
echo "[record] running for ${RUNTIME}s ..."
sleep "${RUNTIME}"

# --- rosbag 정상 종료(SIGINT) ---
echo "[record] stopping recorder (SIGINT) ..."
kill -2 "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" 2>/dev/null || true

# --- dmesg tail (로컬 로그) ---
have_cmd dmesg && dmesg | tail -n 200 > "${OUTDIR}/dmesg_tail.txt" 2>&1 || true

echo "[record] finished."
echo "[record] logs: ${OUTDIR}"
echo "[record] bag : ${REC_DIR}"
