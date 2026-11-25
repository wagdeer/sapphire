#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 4 ]]; then
  printf 'usage: %s sapphire_nclt_replay hits.bin imu.csv output\n' "$0" >&2
  exit 2
fi
cfg="$(cd "$(dirname "$0")/.." && pwd)/cfg"
mkdir -p "$4"
export REPLAY_RATE=${REPLAY_RATE:-12}
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
export OMP_THREAD_LIMIT=${OMP_THREAD_LIMIT:-2}
"$1" main "$cfg/nclt.toml" "$2" "$3" "$4/main" 5634
"$1" methodB "$cfg/cfg_b/nclt.toml" "$2" "$3" "$4/methodB" 5634
