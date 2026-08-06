# Flight log book

One row per flight, filled in per [`docs/FLIGHT_TEST_PLAN.md`](FLIGHT_TEST_PLAN.md) §4.2;
log files are named per §4.1 (`out/logs/<YYYYMMDD>_<FT-id>_f<nn>.csv`). Aborted flights get
rows too — aborted-flight data is often the most valuable.

## Flights

| Date | FT id | Flight # | Log file | Pack ID | V start (rest) | V end (load) | Cmd (script + args) | Firmware | Controller (PID/Mellinger) | Height [m] | Speed avg/peak [m/s] | RMS [m] | Max [m] | Result (PASS/FAIL/ABORT) | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |

## Battery table

From FT-0.3, updated by FT-4.1.

| Pack ID | Purchase date | Resting V full | Idle droop | Measured endurance | Status (active/retired) |
| --- | --- | --- | --- | --- | --- |
|  |  |  |  |  |  |

## Configuration table

One row per change, so any flight's full configuration is reconstructible from its date.

| Date | Firmware version | cflib version | `stabilizer.controller` value | Floor/lighting setup |
| --- | --- | --- | --- | --- |
|  |  |  |  |  |
