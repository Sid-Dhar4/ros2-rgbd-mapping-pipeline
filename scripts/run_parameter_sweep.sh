#!/usr/bin/env bash
set -euo pipefail

# Short reproducible parameter sweep for the RGB-D mapping + occupancy-grid pipeline.
#
# This script runs a few bounded-duration ROS 2 launches, saves local logs/maps
# under outputs/sweeps/, and writes curated summary artifacts under outputs/metrics/.
#
# It is intentionally small: enough to show engineering tradeoffs without turning
# the repo into a large benchmarking framework.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"

DURATION_SECONDS="${DURATION_SECONDS:-35}"
REPLAY_FPS="${REPLAY_FPS:-10.0}"
PUBLISH_EVERY="${PUBLISH_EVERY:-2}"

SWEEP_DIR="${REPO_ROOT}/outputs/sweeps"
METRICS_DIR="${REPO_ROOT}/outputs/metrics"
CSV_PATH="${METRICS_DIR}/parameter_sweep.csv"
SUMMARY_PATH="${METRICS_DIR}/parameter_sweep_summary.md"

mkdir -p "${SWEEP_DIR}" "${METRICS_DIR}"

# ROS setup files may reference optional unset variables, so temporarily
# disable nounset while sourcing them, then restore strict mode.
set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

cat > "${CSV_PATH}" <<CSV
case_name,map_voxel_size_m,accumulate_every_n_frames,publish_every_n_processed_frames,occupancy_resolution_m,expected_update_hz,final_map_points,last_occupied_cells,pcd_size_kb,duration_seconds,notes
CSV

run_case() {
  local case_name="$1"
  local map_voxel="$2"
  local accumulate_every="$3"
  local occupancy_resolution="$4"
  local notes="$5"

  local case_dir="${SWEEP_DIR}/${case_name}"
  local log_path="${case_dir}/run.log"
  local output_basename="final_map_${case_name}"
  local pcd_path="${case_dir}/${output_basename}.pcd"

  rm -rf "${case_dir}"
  mkdir -p "${case_dir}"

  local expected_update_hz
  expected_update_hz="$(python3 - <<PY
replay_fps = float("${REPLAY_FPS}")
accumulate_every = float("${accumulate_every}")
publish_every = float("${PUBLISH_EVERY}")
print(f"{replay_fps / accumulate_every / publish_every:.3f}")
PY
)"

  echo
  echo "========== RUN ${case_name} =========="
  echo "map_voxel_size_m=${map_voxel}"
  echo "accumulate_every_n_frames=${accumulate_every}"
  echo "occupancy_resolution_m=${occupancy_resolution}"
  echo "expected_update_hz=${expected_update_hz}"
  echo "duration_seconds=${DURATION_SECONDS}"

  set +e
  timeout --signal=INT "${DURATION_SECONDS}" \
    ros2 launch rgbd_mapping_pipeline occupancy_grid_pipeline.launch.py \
      map_voxel_size_m:="${map_voxel}" \
      accumulate_every_n_frames:="${accumulate_every}" \
      publish_every_n_processed_frames:="${PUBLISH_EVERY}" \
      resolution_m:="${occupancy_resolution}" \
      output_dir:="${case_dir}" \
      output_basename:="${output_basename}" \
    > "${log_path}" 2>&1
  local status=$?
  set -e

  if [[ "${status}" -ne 0 && "${status}" -ne 124 && "${status}" -ne 130 ]]; then
    echo "ERROR: launch failed for ${case_name}. See ${log_path}"
    tail -n 80 "${log_path}" || true
    exit "${status}"
  fi

  local final_map_points="NA"
  if [[ -f "${pcd_path}" ]]; then
    final_map_points="$(awk '/^POINTS / {print $2}' "${pcd_path}" | tail -n 1)"
  fi

  local last_occupied_cells="NA"
  last_occupied_cells="$(
    grep -o 'occupied cells=[0-9]*' "${log_path}" \
      | tail -n 1 \
      | sed 's/occupied cells=//' \
      || true
  )"
  if [[ -z "${last_occupied_cells}" ]]; then
    last_occupied_cells="NA"
  fi

  local pcd_size_kb="NA"
  if [[ -f "${pcd_path}" ]]; then
    pcd_size_kb="$(du -k "${pcd_path}" | awk '{print $1}')"
  fi

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,"%s"\n' \
    "${case_name}" \
    "${map_voxel}" \
    "${accumulate_every}" \
    "${PUBLISH_EVERY}" \
    "${occupancy_resolution}" \
    "${expected_update_hz}" \
    "${final_map_points}" \
    "${last_occupied_cells}" \
    "${pcd_size_kb}" \
    "${DURATION_SECONDS}" \
    "${notes}" \
    >> "${CSV_PATH}"

  echo "Wrote case summary for ${case_name}"
}

run_case "baseline" "0.05" "3" "0.05" "Default V1.2-style settings."
run_case "dense_map" "0.03" "3" "0.05" "Smaller map voxel keeps more geometric detail."
run_case "faster_sparse" "0.08" "1" "0.10" "Sparse map with faster expected updates and coarser occupancy grid."

cat > "${SUMMARY_PATH}" <<MD
# Parameter Sweep Summary

This sweep runs short live launches of the RGB-D mapping plus occupancy-grid pipeline.

## What is varied

- map voxel size
- accumulation interval
- occupancy-grid resolution

## Generated artifacts

- \`outputs/metrics/parameter_sweep.csv\`
- local raw logs/maps under \`outputs/sweeps/\` ignored by git

## Run command

\`\`\`bash
DURATION_SECONDS=${DURATION_SECONDS} bash scripts/run_parameter_sweep.sh
\`\`\`

## Notes

- The sweep is intentionally short and practical for local verification.
- \`expected_update_hz\` is computed from replay FPS, accumulation interval, and publish interval.
- \`final_map_points\` is parsed from the generated PCD header.
- \`last_occupied_cells\` is parsed from the occupancy-grid node logs.
- Raw per-case logs and generated maps are local artifacts and are not committed.
MD

echo
echo "========== SWEEP COMPLETE =========="
echo "CSV: ${CSV_PATH}"
echo "Summary: ${SUMMARY_PATH}"
echo
cat "${CSV_PATH}"
