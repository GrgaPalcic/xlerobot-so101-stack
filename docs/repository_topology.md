# Repository And Machine Topology

This document is the source of truth for which SO-101 folders mean what across
the Dell robot host and the local/GPU development machine.

The short version: use one Git repository for source code, keep calibration run
artifacts under ignored run directories, and always pass an explicit
`--workspace` and `--out` to the XLeRobot calibration CLI.

## Current Machine Roles

```text
local/GPU machine
  /home/grga/Documents/so101-ros-physical-ai
    Current development checkout. This is where code/docs are edited first.

  /home/grga/Documents/so101-custom
    Historical mistake path. It is not a git repository and currently only
    contains tmp.zip. Do not use it as a source of truth.

Dell robot host
  /home/dell/Documents/so101-ros-physical-ai
    Original Dell field checkout. This accumulated the one-arm local
    integration, camera/grasp experiments, and many uncommitted/untracked field
    files. Treat it as a legacy source/backup until it is reconciled.

  /home/dell/Documents/so101-ros-physical-ai-xlerobot-calib
    Separate git worktree of the same SO-101 repository, checked out on the
    xlerobot/calibration-cli branch. This is not a different repository. Use
    this for the current two-arm calibration run until a shared remote fork is
    configured and all machines can clone/pull the same branch.

  /home/dell/Documents/lerobot
    LeRobot checkout used for SO-101 motor setup and LeRobot arm calibration.
    This is not the ROS stack.

  /home/dell/Documents/lerobot-calib
    LeRobot-generated calibration JSONs and related calibration state. Preserve
    this; copy selected outputs into run artifacts or generated ROS joint YAMLs
    when needed.

  /home/dell/Documents/so101_calibration_backup_*
    Backups of prior Dell calibration outputs. Keep these until the new
    two-arm calibration has been validated end to end.
```

Last verified Dell state: 2026-05-22 03:15 CEST.

## Repository Vs Branch Vs Worktree

These three concepts caused most of the confusion:

```text
repository
  The whole git project: history, branches, tags, files.

branch
  A named line of development inside that repository.
  xlerobot/calibration-cli is a branch inside the SO-101 repository.

worktree
  A second folder checked out from the same repository object database.
  Dell uses this so main can remain in /home/dell/Documents/so101-ros-physical-ai
  while xlerobot/calibration-cli is checked out in
  /home/dell/Documents/so101-ros-physical-ai-xlerobot-calib.
```

So `xlerobot/calibration-cli` is not another repo. It is our active branch in
the SO-101 repo. It only looks separate on Dell because it has its own folder.

## Current Branch State

Observed in `/home/grga/Documents/so101-ros-physical-ai` on 2026-05-22:

```text
xlerobot/calibration-cli
  Active local branch for the two-arm calibration CLI, printed ChArUco plates,
  and current documentation cleanup.

xlerobot/two-arm-calibration-runbook
  Earlier branch containing the detailed runbook. Its work is included in
  xlerobot/calibration-cli.

archive/dell-dual-camera-grasping-2026-05-17
  Archive of the dirty Dell one-arm dual-camera/grasping integration before
  the clean XLeRobot runbook/CLI work started.

main
  Local main from the original upstream stack. Keep it as an upstream tracking
  branch, not as the active lab branch.

origin/main
  Upstream GitHub remote at https://github.com/legalaspro/so101-ros-physical-ai.
  It has newer inference-side commits than local main.
```

Observed on Dell on 2026-05-22:

```text
/home/dell/Documents/so101-ros-physical-ai
  branch: main
  state: dirty legacy field checkout with many modified/untracked files

/home/dell/Documents/so101-ros-physical-ai-xlerobot-calib
  branch: xlerobot/calibration-cli
  state: dirty current calibration worktree
  note: behind the local/GPU checkout until the latest commits are pushed or
        transferred again

/home/dell/Documents/lerobot
  branch: main
  state: dirty LeRobot checkout with local record script/edit

/home/dell/Documents/lerobot-calib
  not a git repository
```

The active `xlerobot/calibration-cli` branch has not been pushed to a shared
remote yet. Dell received it via a git bundle, which is useful short-term but
should not be the long-term synchronization method.

## Recommended Remote Layout

Create or choose one lab-owned fork, then use it on every machine. This should
be a fork of `legalaspro/so101-ros-physical-ai`, not a separate unrelated repo,
unless the goal is to permanently sever upstream history.

Recommended remotes:

```bash
git remote rename origin upstream
git remote add origin git@github.com:GrgaPalcic/so101-ros-physical-ai.git
git fetch --all --prune
```

Recommended branch policy:

```text
main
  Mirrors upstream/main. Do not put local lab calibration work here.

xlerobot/calibration-cli
  Active two-arm branch until the first clean calibration and dry perception
  pass are complete.

xlerobot/main
  Optional future stable lab branch after xlerobot/calibration-cli is validated.

archive/*
  Historical snapshots. Do not build daily work on these branches.
```

After the fork exists:

```bash
git push -u origin xlerobot/calibration-cli
git push origin archive/dell-dual-camera-grasping-2026-05-17
```

Then on Dell and GPU/inference hosts:

```bash
git clone git@github.com:GrgaPalcic/so101-ros-physical-ai.git
cd so101-ros-physical-ai
git checkout xlerobot/calibration-cli
git submodule update --init --recursive
```

The local shell is not currently logged into GitHub through `gh`, so fork
creation/pushing still needs either `gh auth login`, GitHub UI setup, or a
pre-existing SSH remote URL.

If a completely new GitHub repository is preferred instead of a fork, keep the
same branch policy but create a new empty repo first and set it as `origin`.
That is workable, but it loses GitHub's explicit fork relationship and makes
upstream merges slightly more manual.

## Why The Wizard Started From Scratch

`xlerobot-calib` resumes by searching this path:

```text
<workspace>/field_runs/xlerobot_*/run_state.yaml
```

If you run it from a different checkout, or pass a different `--workspace`, it
looks in a different `field_runs/` directory and appears to start over.

If you pass `--out`, that exact run directory is used:

```text
<out>/run_state.yaml
```

Always pin both variables on Dell:

```bash
export XLEROBOT_WS=/home/dell/Documents/so101-ros-physical-ai-xlerobot-calib
export XLEROBOT_RUN=$XLEROBOT_WS/field_runs/xlerobot_printed_plate_20260520

cd "$XLEROBOT_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run xlerobot_calibration xlerobot-calib \
  --workspace "$XLEROBOT_WS" \
  --out "$XLEROBOT_RUN" \
  status
```

Use the same `XLEROBOT_RUN` for `wizard`, `status`, `show-config`, and
`run-step`.

## Calibration Data Policy

Keep raw and generated run data out of git:

```text
field_runs/
calibration_runs/
install/
build/
log/
```

Commit only curated source files, scripts, docs, and known-good config YAMLs.
For calibration results, prefer this flow:

```text
1. Capture into field_runs/xlerobot_<RUN_ID>/...
2. Export a report from the CLI.
3. Review quality numbers and overlays.
4. Promote only selected stable YAMLs into so101_bringup/config/... if needed.
5. Preserve old runs as tarballs/backups until the replacement is validated.
```

## Current Stack Scope

Keep using these as active stack areas:

```text
so101_bringup/
so101_description/
so101_teleop/
so101_kinematics/
so101_moveit_config/
so101_grasp_msgs/
so101_grasping/
grasp_server/
scripts/
xlerobot_calibration/
docs/
```

Keep but do not prioritize unless needed:

```text
episode_recorder/
rosbag_to_lerobot/
so101_inference/
policy_server/
so101_camera_calibration/
```

Do not edit the third-party submodule unless explicitly working on the driver:

```text
feetech_ros2_driver/
```

## Refresh Commands

Local machine:

```bash
cd /home/grga/Documents/so101-ros-physical-ai
git status --short --branch
git branch -vv
git branch -a --format='%(refname:short) %(objectname:short) %(upstream:short) %(subject)'
git remote -v
git worktree list
```

Dell:

```bash
ssh dell@192.168.1.73 '
  for repo in \
    /home/dell/Documents/so101-ros-physical-ai \
    /home/dell/Documents/so101-ros-physical-ai-xlerobot-calib \
    /home/dell/Documents/lerobot \
    /home/dell/Documents/lerobot-calib
  do
    echo "--- $repo ---"
    if git -C "$repo" rev-parse --git-dir >/dev/null 2>&1; then
      git -C "$repo" status --short --branch
      git -C "$repo" branch -vv
      git -C "$repo" remote -v
      git -C "$repo" worktree list
    elif [ -d "$repo" ]; then
      echo "not a git repo"
      find "$repo" -maxdepth 2 -type f | sed -n "1,40p"
    else
      echo "missing"
    fi
  done
'
```
