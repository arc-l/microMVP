"""Hardware bring-up checks.

Small scripts you run by hand against real hardware, to confirm one layer
works before debugging the one above it. Not part of the test suite —
they need a gateway and cars plugged in, and a person watching.

    python -m hardware_test.find_ap        # which serial port is the gateway
    python -m hardware_test.check_motion --cars 3   # do the wheels turn
"""
