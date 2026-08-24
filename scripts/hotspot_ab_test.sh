#!/bin/bash
# Sapphire hotspot A/B verification test
# Compares baseline vs optimized configurations
set -euo pipefail

BAG=/data/rosbag2_2024_04_16-14_17_01
TOML_DIR=/workspace/src/sapphire/cfg
BIN=/workspace/install/sapphire_ros2/lib/sapphire_ros2/sapphire_ros_node
RESULTS=/tmp/hotspot_tests
COMMON_ARGS="--ros-args -r lidar/points:=/livox/lidar -r imu/data:=/livox/imu -r __node:=sapphire_test"

mkdir -p "$RESULTS"

run_test() {
    local name=$1
    local config=$2
    local env_vars=$3
    local prof="$RESULTS/${name}.prof"

    echo ""
    echo "=== TEST: $name ==="
    
    # Clean up previous
    pkill -f sapphire_test 2>/dev/null || true
    sleep 1
    rm -f "$prof"

    # Launch with profiler
    export CPUPROFILE="$prof"
    $env_vars $BIN $COMMON_ARGS -p config_file:="$TOML_DIR/$config" \
        > "$RESULTS/${name}_stdout.log" 2> "$RESULTS/${name}_stderr.log" &
    PID=$!
    sleep 4

    # Play bag
    ros2 bag play "$BAG" --clock 100 --read-ahead-queue-size 1000 \
        > "$RESULTS/${name}_bag.log" 2>&1 || true
    
    sleep 2
    kill -INT $PID 2>/dev/null
    sleep 3
    kill -0 $PID 2>/dev/null && kill -KILL $PID 2>/dev/null
    sleep 1

    if [ -s "$prof" ]; then
        echo "  Samples: $(google-pprof --text $BIN $prof 2>/dev/null | head -1 | awk '{print $1}')"
        echo "  Top 5:"
        google-pprof --text --functions $BIN $prof 2>/dev/null | head -6 | tail -5 | \
            awk '{printf "    %6s  %s\n", $1, $5}'
    else
        echo "  FAILED: profile empty"
    fi
}

echo "=== Sapphire Hotspot A/B Tests ==="
echo "Bag: $BAG"
echo ""

# Test 1: Baseline (current config)
run_test "baseline" "sapphire_mid360.toml" ""

# Test 2: OMP_WAIT_POLICY=active
run_test "omp_active" "sapphire_mid360.toml" \
    "OMP_WAIT_POLICY=active OMP_DYNAMIC=false"

# Test 3: No occupancy (use observer config which has no occupancy section)
# Note: need to copy and modify the config
cp "$TOML_DIR/sapphire_mid360.toml" "$TOML_DIR/sapphire_mid360_no_occ.toml"
sed -i 's/enabled = true/enabled = false/' "$TOML_DIR/sapphire_mid360_no_occ.toml"
run_test "no_occupancy" "sapphire_mid360_no_occ.toml" ""

# Test 4: Combined (OMP + no occupancy)
run_test "combined" "sapphire_mid360_no_occ.toml" \
    "OMP_WAIT_POLICY=active OMP_DYNAMIC=false"

echo ""
echo "=== SUMMARY ==="
for f in "$RESULTS"/*.prof; do
    name=$(basename "$f" .prof)
    samples=$(google-pprof --text $BIN "$f" 2>/dev/null | head -1 | awk '{print $2}')
    echo "  $name: $samples"
done

echo ""
echo "Results in: $RESULTS/"
