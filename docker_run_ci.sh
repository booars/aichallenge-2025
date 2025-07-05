#!/bin/bash
#GITHUB_WORKSPACE=$(pwd)
mkdir -p output

LOG_DIR="output/latest"
mkdir -p $LOG_DIR
docker run --rm -it \
  --name aichallenge-2025 \
  --network host \
  --privileged \
  -v ${GITHUB_WORKSPACE}/aichallenge:/aichallenge \
  -e DISPLAY -e TERM \
  "aichallenge-2025-dev-${USER}" \
  bash -c "cd /aichallenge && ./build_autoware.bash && source /aichallenge/workspace/install/setup.bash && ./run_evaluation.bash"