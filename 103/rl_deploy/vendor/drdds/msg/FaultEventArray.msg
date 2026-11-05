# Complete snapshot of currently active faults.
# An empty active_faults array means that no faults are currently active.

builtin_interfaces/Time timestamp       # Time when this snapshot was generated
drdds/FaultEvent[] active_faults        # Faults active at the snapshot time
