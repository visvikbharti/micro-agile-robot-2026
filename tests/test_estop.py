"""Tests for flight/estop.py: the panic button's dry-run path, with no cflib."""
from __future__ import annotations

import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "flight"))

import estop  # noqa: E402


def test_import_does_not_pull_cflib():
    assert not any(m == "cflib" or m.startswith("cflib.") for m in sys.modules)


def test_default_uri_matches_flight_scripts():
    import fly_hover  # noqa: E402  (same directory, same convention)

    assert estop.DEFAULT_URI == fly_hover.DEFAULT_URI == "radio://0/80/2M/E7E7E7E7E7"


# --- Dry run ---------------------------------------------------------------------


def test_dry_run_exits_zero_without_cflib(capsys):
    rc = estop.main(["--dry-run"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "Dry run" in out
    assert estop.DEFAULT_URI in out
    assert "emergency stop" in out.lower()
    assert not any(m == "cflib" or m.startswith("cflib.") for m in sys.modules)


def test_dry_run_honors_custom_uri(capsys):
    uri = "radio://0/100/2M/AABBCCDDEE"
    assert estop.main(["--dry-run", "--uri", uri]) == 0
    assert uri in capsys.readouterr().out


def test_rejects_unknown_argument():
    with pytest.raises(SystemExit) as excinfo:
        estop.main(["--dry-run", "--frobnicate"])
    assert excinfo.value.code == 2


def test_dry_run_subprocess_needs_no_cflib():
    script = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "flight", "estop.py")
    proc = subprocess.run([sys.executable, script, "--dry-run"],
                          capture_output=True, text=True)
    assert proc.returncode == 0
    assert "Dry run" in proc.stdout
