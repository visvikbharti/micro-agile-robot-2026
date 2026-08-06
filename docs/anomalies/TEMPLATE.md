# Anomaly <YYYYMMDD> — <FT-id> — <one-line title>

<!--
Copy this file to docs/anomalies/<YYYYMMDD>_<FT-id>_<slug>.md (naming convention from
docs/FLIGHT_TEST_PLAN.md §4.3), fill in every field, delete this comment. A crash or
safety near-miss always gets a report before the next powered test, even props-off;
three same-card failures also trigger one (§1.1). Write it the same day.
-->

- **Flight**: log file(s), log-book row(s)
- **Severity**: note / abort / crash / hardware damage / safety near-miss
- **What was commanded**: script + args, reference CSV
- **What happened**: factual timeline, times from the log (not from memory)
- **What the data shows**: the log evidence — variables, values, plots
- **Candidate causes**: each mapped, where possible, to a DESIGN §9 delta or ch07
  mechanism (drag, motor lag, sag, estimator bias/drift/latency, ground effect, yaw
  authority) — or "unknown", honestly
- **Ruled out**: what you checked that it wasn't
- **Action**: fix / procedure change / threshold change (with re-derivation) / accept
- **Re-test**: which card re-flies, and its result
