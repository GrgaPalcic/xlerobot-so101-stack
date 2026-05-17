# rosbag_to_lerobot Agent Notes

This package converts MCAP episodes into LeRobot datasets.

Important synchronization rule:

```text
reference topic emits dataset frames
all other features are sampled as latest message <= reference timestamp
stale features are dropped
```

Do not introduce future leakage by pairing an observation with a later action.

If camera topics or names change, update:

```text
rosbag_to_lerobot/config/so101.yaml
episode_recorder/config/default_config.yaml
so101_inference camera observation names
```

