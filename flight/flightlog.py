"""Stream Crazyflie state to an ``analyze_log.py``-ready CSV during a flight.

``CsvFlightLogger`` wraps a connected ``SyncCrazyflie`` and logs
``stateEstimate.x/y/z`` plus ``pm.vbat`` in ONE log block at 50 Hz (4 floats =
16 bytes, comfortably inside a CRTP log packet's payload) to a CSV with header
``t,x,y,z,vbat`` and ``t`` in SECONDS relative to the first sample — exactly
the flight-log format ``flight/analyze_log.py`` documents. In-script logging
exists because cfclient cannot share the Crazyradio link with a running
script; the GUI's logging tab is a fallback for manual sessions only.

Usage, inside an open ``SyncCrazyflie`` context — either as a context manager::

    with CsvFlightLogger(scf, "out/ft11_log.csv"):
        ...  # arm, take off, fly, land

or via the explicit ``logger.start()`` / ``logger.stop()`` pair (``stop()`` is
idempotent, safe before ``start()`` and in a ``finally:`` block).

The pure CSV logic (ms-timestamp conversion, row formatting, writing) lives in
``FlightLogWriter``, which needs neither cflib nor hardware and is unit-tested
with synthetic callback data; ``CsvFlightLogger`` adds only the cflib wiring
and imports cflib lazily inside ``start()``.
"""

from __future__ import annotations

import os

LOG_VARIABLES = ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "pm.vbat")
LOG_PERIOD_MS = 20  # 50 Hz; 4 floats = 16 bytes fits one CRTP log packet
CSV_HEADER = "t,x,y,z,vbat"


class FlightLogWriter:
    """Pure CSV-writing state: header, t0 capture, ms-to-s conversion, formatting.

    Feed it log-callback samples via ``add(timestamp_ms, data)`` where ``data``
    maps each name in ``LOG_VARIABLES`` to a float. No cflib, no I/O beyond the
    file object handed in — unit-testable with synthetic data.
    """

    def __init__(self, fileobj):
        self._f = fileobj
        self._t0_ms = None
        self.n_rows = 0
        self._f.write(CSV_HEADER + "\n")

    def add(self, timestamp_ms, data):
        """Append one row; ``timestamp_ms`` is the callback's onboard ms clock."""
        if self._t0_ms is None:
            self._t0_ms = timestamp_ms
        t = (timestamp_ms - self._t0_ms) / 1000.0
        self._f.write("%.3f,%.6f,%.6f,%.6f,%.3f\n"
                      % (t,
                         data["stateEstimate.x"],
                         data["stateEstimate.y"],
                         data["stateEstimate.z"],
                         data["pm.vbat"]))
        self._f.flush()  # a crashed script should still leave a scorable log
        self.n_rows += 1


class CsvFlightLogger:
    """Log ``LOG_VARIABLES`` from a connected ``SyncCrazyflie`` to a CSV file.

    Constructing the object touches no cflib; only ``start()`` does. Use as a
    context manager or call ``start()`` / ``stop()`` explicitly (see module
    docstring).
    """

    def __init__(self, scf, path):
        self._scf = scf
        self.path = path
        self._file = None
        self._writer = None
        self._log_config = None

    def start(self):
        """Open the CSV, start the 50 Hz log block, and return ``self``."""
        from cflib.crazyflie.log import LogConfig

        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._file = open(self.path, "w")
        self._writer = FlightLogWriter(self._file)
        log_config = LogConfig(name="flightlog", period_in_ms=LOG_PERIOD_MS)
        for var in LOG_VARIABLES:
            log_config.add_variable(var, "float")
        log_config.data_received_cb.add_callback(self._on_data)
        self._scf.cf.log.add_config(log_config)
        log_config.start()
        self._log_config = log_config
        print("Logging %s at %d Hz to %s"
              % (", ".join(LOG_VARIABLES), 1000 // LOG_PERIOD_MS, self.path))
        return self

    def _on_data(self, timestamp, data, logconf):
        """cflib log callback: delegate to the pure writer."""
        writer = self._writer
        if writer is not None:
            writer.add(timestamp, data)

    def stop(self):
        """Stop the log block and close the CSV. Idempotent."""
        if self._log_config is not None:
            try:
                self._log_config.stop()
            finally:
                self._log_config = None
        writer, self._writer = self._writer, None
        if self._file is not None:
            f, self._file = self._file, None
            f.close()
            print("Wrote %d log rows to %s"
                  % (writer.n_rows if writer is not None else 0, self.path))

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False
