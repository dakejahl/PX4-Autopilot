# Raw flow capture

`CONFIG_PAA3905_RAW_DEBUG=y` is enabled on this branch for `ark_can-flow-mr_default` and `ark_fmu-v6x_default`. Build and flash both. On the FC, enable `UAVCAN_SUB_FLOW` and normal logging (`SDLOG_PROFILE` includes Default; `SDLOG_MODE=1` also records disarmed bench tests). Verify `listener flow_raw`, `logger status`, and `uavcan status` before recording.

| Topic/field | Meaning |
|---|---|
| `paa3905_raw` | Every successful SPI burst, including rejected and startup readings; local SPI logging |
| `flow_raw` | Raw counts plus gyro integral; CAN reception or local SPI instance 0 |
| `timestamp_sample` | Host burst-read start in FC HRT; interval ends here |
| `node_timestamp_us` / `bus_timestamp_us` | Original board HRT / synchronized CAN acquisition time |
| `timestamp_sample_valid` | Local timestamp or plausible CAN clock mapping; false means receive-time fallback |
| `interval_us=0` | Unknown window after reset/SPI error |
| `gyro_samples=0`, NaN integral | Incomplete gyro coverage, queue loss, or source change |
| `frame_counter` | Successful reads; discontinuities expose capture loss or reboot |

Counts precede software rotation/scaling. Gyro is uncalibrated `sensor_gyro`, integrated with interpolated endpoints in source-board FRD. Native chip exposure and tracking latency remain unmeasured. `SENS_FLOW_RATE` does not reduce diagnostic traffic: experimental type ID 20090 sends 46 bytes, seven CAN frames per burst, at lowest priority. Check counters for bus saturation.

Install `numpy`, `pyulog`, and `matplotlib`. For the brief's two 180° rotations:

```sh
python3 Tools/flow/compare_raw_flow.py flight.ulg --output results \
  --node-yaw-deg 180 --raw-yaw-deg 0
```

Select topic instances and supply `--flow-minus-gnss X Y Z` for the measured camera-minus-antenna offset. Output: every raw frame in CSV, fit/held-out north/east errors in JSON, and plots. The model assumes yaw-only mounts and range along the camera axis. GNSS vertical velocity is held fixed during horizontal reconstruction. Baseline uses the nominal scale at the fitted reference shift. The timing scan shifts GNSS/attitude/range relative to the captured gyro; it cannot identify camera/gyro latency independently. Use GNSS sample timestamps; receive-time correction is explicit. Refit under several light/motion conditions before changing estimator gains or gates.
