#!/usr/bin/env bash
set -euo pipefail

OWNER="HARSHAVARDHAN-SEKAR"
REPO="ros2-drone-sar-autonomy"
DESCRIPTION="Simulation-first autonomous SAR drone stack with PX4, ROS 2, Gazebo Harmonic, MAVROS, OctoMap, frontier exploration, A* planning, depth perception and YOLO victim-candidate detection."
TOPICS=(ros2 px4 gazebo mavros uav drone autonomous-drone octomap path-planning computer-vision yolo robotics search-and-rescue)

cd "$(dirname "$0")/.."

if ! command -v gh >/dev/null 2>&1; then
  echo "GitHub CLI (gh) is not installed. Install/authenticate gh or create an empty repository named $REPO in the GitHub UI, then follow docs/GITHUB_PUBLISH.md." >&2
  exit 2
fi

gh auth status
./scripts/run_checks.sh

if [[ ! -d .git ]]; then
  git init -b main
fi

git config user.name "Harshavardhan Coimbatore Sekar"

# Preserve an already-configured email. Otherwise use GitHub's authenticated
# account email only when available; fall back to the project maintainer email.
if ! git config user.email >/dev/null; then
  git config user.email "harshasekar.09@gmail.com"
fi

git add .
if ! git diff --cached --quiet; then
  git commit -m "feat: add ROS 2 PX4 search-and-rescue autonomy stack"
fi

if gh repo view "$OWNER/$REPO" >/dev/null 2>&1; then
  if git remote get-url origin >/dev/null 2>&1; then
    git remote set-url origin "https://github.com/$OWNER/$REPO.git"
  else
    git remote add origin "https://github.com/$OWNER/$REPO.git"
  fi
  git push -u origin main
else
  gh repo create "$OWNER/$REPO" \
    --public \
    --description "$DESCRIPTION" \
    --source=. \
    --remote=origin \
    --push
fi

for topic in "${TOPICS[@]}"; do
  gh repo edit "$OWNER/$REPO" --add-topic "$topic"
done
echo "Published: https://github.com/$OWNER/$REPO"
