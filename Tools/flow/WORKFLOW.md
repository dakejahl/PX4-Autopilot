# Optical flow: branch workflow

| Branch | Repo | Purpose |
|---|---|---|
| `dakejahl/flow-low-light-reset` | PX4/PX4-Autopilot, [#28632](https://github.com/PX4/PX4-Autopilot/pull/28632) | Every driver, VehicleOpticalFlow and board change meant for upstream |
| `dakejahl/flow-raw-capture` | dakejahl/PX4-Autopilot, [#58](https://github.com/dakejahl/PX4-Autopilot/pull/58) | Testing branch that flies: #28632 plus the raw capture (`flow_raw`, `paa3905_raw`, this tool) |

The capture commits never go upstream. Everything else is written on #28632 and rolled into #58.

## Change

```sh
cd ~/code/jake/PX4-Autopilot-flow-reset            # worktree on dakejahl/flow-low-light-reset
# edit, make format, commit
git push origin dakejahl/flow-low-light-reset
cd ~/code/jake/PX4-Autopilot                       # dakejahl/flow-raw-capture
git cherry-pick -x <sha>
make ark_can-flow-mr_default                       # or ark_fmu-v6x_default for FC-side code
git push jake dakejahl/flow-raw-capture
```

Keep #28632's description current with what its branch now does. Do not open another PR for work in this effort.

## Fly

Flash both sides from the same head, set the vehicle parameters listed in the latest #58 comment, log with `SDLOG_PROFILE` including the default set, then:

```sh
python3 Tools/flow/compare_raw_flow.py <log> --output <dir> --node-yaw-deg 0 --raw-yaw-deg 0 --squal-min 85
```

Write the findings to `~/Downloads/v6xrt/results/<date>-<name>/summary.txt` and post the hand-off as a comment on #58: verified, in this head not yet flown, parameters, next.

## Constraints

- Check the NuttX gitlink against `origin/main` before every push to #28632.
- One build per change.
- The bench is shared; check for other users before flashing.
