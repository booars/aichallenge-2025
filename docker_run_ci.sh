#!/bin/bash

docker run --rm \
  --name aichallenge-2025 \
  --network host \
  --privileged \
  -v ${GITHUB_WORKSPACE}/output:/output \
  -v ${GITHUB_WORKSPACE}/aichallenge:/aichallenge \
  -v ${GITHUB_WORKSPACE}/remote:/remote \
  -v ${GITHUB_WORKSPACE}/vehicle:/vehicle \
  -e DISPLAY -e TERM \
  "aichallenge-2025-dev-${USER}" \
  bash -c "cd /aichallenge && ./build_autoware.bash && ./run_evaluation.bash"