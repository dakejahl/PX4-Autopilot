# Barometer Thrust Compensation Calibration

Tool for calibrating the barometer thrust compensation parameters
(`SENS_BARO_PCOEF` and `SENS_BARO_PTAU`) from flight log data.

## Background

Propwash from propellers changes the static pressure at the barometer sensor.
This creates a thrust-dependent altitude error: as thrust increases, the baro
reading shifts. The direction and magnitude depend on sensor placement relative
to the propellers.

The `vehicle_air_data` module compensates for this by applying a correction to
the barometer altitude before publishing:

```
corrected_baro_alt = raw_baro_alt + SENS_BARO_PCOEF * filtered_thrust
```

where `filtered_thrust` is the normalized thrust magnitude [0, 1] passed through
a first-order low-pass filter with time constant `SENS_BARO_PTAU`. The filter
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

For higher-rate baro data (better calibration resolution), add the
**High rate sensors** logging profile (bit 11) before flight:

```
param set SDLOG_PROFILE 2049
```

### Flight Procedure

1. **Disable existing compensation** before the calibration flight:
   ```
   param set SENS_BARO_PCOEF 0.0
   param set SENS_BARO_PTAU 0.0
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
- **Recommended `SENS_BARO_PCOEF` and `SENS_BARO_PTAU` values**

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
  tau is marked — this becomes `SENS_BARO_PTAU`.
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
   - `SENS_BARO_PCOEF = -K` (negated because the module adds the correction)
   - `SENS_BARO_PTAU = tau`

## Applying Parameters

After calibration, set the parameters on the vehicle:

```
param set SENS_BARO_PCOEF <value>
param set SENS_BARO_PTAU <value>
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
  `SENS_BARO_PTAU = 0` is fine (no filtering needed).
- **Very large K (> 5 m)**: May indicate a sensor mounting issue. The baro
  should be shielded from direct propwash where possible.

## Parameters Reference

| Parameter | Description | Range | Default |
|-----------|-------------|-------|---------|
| `SENS_BARO_PCOEF` | Baro altitude correction per unit thrust [m] | -30 to 30 | 0.0 |
| `SENS_BARO_PTAU` | Thrust filter time constant [s] | 0 to 2 | 0.0 |
