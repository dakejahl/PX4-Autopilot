# Barometer Thrust Compensation Calibration

Tool for calibrating the EKF2 barometer thrust compensation parameters
(`EKF2_PCOEF_THR` and `EKF2_PCOEF_TTAU`) from flight log data.

## Background

On multirotor vehicles, propwash from the rotors changes the static pressure at
the barometer sensor. This creates a thrust-dependent altitude error: as thrust
increases, the baro reading shifts (typically reading higher altitude due to
pressure depression, though the direction depends on sensor placement).

The EKF2 can compensate for this by applying a correction to the barometer
measurement:

```
corrected_baro_alt = raw_baro_alt + EKF2_PCOEF_THR * filtered_thrust
```

where `filtered_thrust` is the normalized thrust magnitude [0, 1] passed through
a first-order low-pass filter with time constant `EKF2_PCOEF_TTAU`. The filter
models the delay between a thrust setpoint change and the resulting pressure
change at the sensor (motor spin-up, propwash development, sensor response).

This tool identifies the optimal values for both parameters by comparing
barometer altitude against a range sensor (ground truth) during hover flight.

## Prerequisites

- **Range sensor**: A downward-facing lidar or sonar is required as the altitude
  ground truth. The range sensor must be logging `distance_sensor`.
- **Python packages**: `pyulog`, `numpy`, `scipy`, `matplotlib`

  ```bash
  pip install pyulog numpy scipy matplotlib
  ```

## Data Collection

### Logging

The default PX4 logging profile includes all required topics:
- `vehicle_air_data` (barometer altitude)
- `distance_sensor` (range sensor)
- `vehicle_thrust_setpoint` (thrust)

For higher-rate baro data (better calibration resolution), enable the
**high rate** logging profile before flight:

```
param set SDLOG_PROFILE 3
```

This adds high-rate `vehicle_air_data` and `estimator_aid_src_baro_hgt` logging.

### Flight Procedure

1. **Disable existing compensation** before the calibration flight:
   ```
   param set EKF2_PCOEF_THR 0.0
   param set EKF2_PCOEF_TTAU 0.0
   ```

2. **Fly a hover** at 2-5 m AGL (above ground effect height) for at least 60
   seconds. Include some gentle altitude changes and throttle variation — the
   calibration needs thrust variation to identify the relationship. Avoid
   aggressive maneuvers.

3. **Land and download** the `.ulg` log file.

## Usage

```bash
cd PX4-Autopilot/Tools/baro_compensation

python3 baro_thrust_calibration.py <path/to/log.ulg>

# Save output to a specific directory:
python3 baro_thrust_calibration.py <path/to/log.ulg> --output-dir /tmp/results
```

## Output

### Console

The tool prints a summary including:
- Baro error statistics (mean offset and standard deviation vs range sensor)
- Thrust-baro correlation strength
- Identified model parameters (gain K, time constant tau, R²)
- **Recommended `EKF2_PCOEF_THR` and `EKF2_PCOEF_TTAU` values**

### PDF Report (`baro_calibration.pdf`)

**Page 1 — Data Overview**:
- Baro altitude vs range sensor altitude over time
- Baro error timeseries (baro minus range sensor)
- Thrust magnitude timeseries

The gap between baro and range sensor lines shows the pressurization error. If
the error tracks thrust changes, compensation will help.

**Page 2 — Calibration Results**:
- **Cross-correlation**: Shows the delay structure between thrust and baro error.
  A peak at positive lag means the baro error lags behind thrust changes.
- **R² vs tau**: Shows model fit quality across filter time constants. The best
  tau is marked — this becomes `EKF2_PCOEF_TTAU`.
- **Before/after**: Raw baro error vs compensated error during hover. The
  compensated trace should be closer to zero.
- **Recommended parameters**: The identified values to set.

**Page 3 — Scatter Plots**:
- Raw baro error vs thrust: shows the linear relationship (slope ≈ K)
- Compensated error vs thrust: should show reduced correlation after compensation

## How the Algorithm Works

1. **Baro error computation**: The baro altitude (zeroed at arm time) is
   compared against the range sensor to get `baro_error = baro_alt - range_dist`
   at each range sensor sample.

2. **Hover extraction**: The middle 60% of the armed period is used as the
   analysis window, avoiding takeoff/landing transients.

3. **Detrending**: A linear trend is removed from the baro error to isolate
   thrust-induced variation from slow thermal drift.

4. **Cross-correlation**: Computed between detrended baro error and detrended
   thrust to visualize the delay structure.

5. **Grid search**: For each candidate time constant tau:
   - Apply a first-order LPF to the thrust signal: `alpha = dt / (dt + tau)`
   - Fit `baro_error = K * filtered_thrust + c` via least-squares
   - Compute R² (fraction of error variance explained by thrust)

6. **Best model**: The tau with highest R² is selected. The gain K and tau
   become the recommended parameters:
   - `EKF2_PCOEF_THR = -K` (negated because the EKF adds the correction)
   - `EKF2_PCOEF_TTAU = tau`

## Applying Parameters

After calibration, set the parameters on the vehicle:

```
param set EKF2_PCOEF_THR <value>
param set EKF2_PCOEF_TTAU <value>
```

Then fly again and re-run the tool on the new log to verify the compensation is
working. The baro error should show reduced correlation with thrust.

## Interpreting Results

| Metric | Good | Marginal | Poor |
|--------|------|----------|------|
| Thrust correlation \|r\| | > 0.6 | 0.3 - 0.6 | < 0.3 |
| Model R² | > 0.3 | 0.1 - 0.3 | < 0.1 |
| Compensated \|r\| | < 0.2 | 0.2 - 0.4 | > 0.4 |

- **Low R²**: Thrust is not the dominant baro error source. Consider thermal
  drift, ground effect, or sensor placement issues.
- **R² improvement from lag ≈ 0**: The system delay is short enough that
  `EKF2_PCOEF_TTAU = 0` is fine (no filtering needed).
- **Very large K (> 5 m)**: May indicate a sensor mounting issue. The baro
  should be shielded from direct propwash where possible.

## Parameters Reference

| Parameter | Description | Range | Default |
|-----------|-------------|-------|---------|
| `EKF2_PCOEF_THR` | Baro altitude correction per unit thrust [m] | -10 to 10 | 0.0 |
| `EKF2_PCOEF_TTAU` | Thrust filter time constant [s] | 0 to 2 | 0.0 |
