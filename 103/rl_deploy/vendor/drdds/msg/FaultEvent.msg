# Fault event reported by the robot diagnostic system.

# Event types
uint8 TYPE_START  = 1
uint8 TYPE_STOP   = 2

# Severity levels
uint8 SEVERITY_DEBUG    = 0
uint8 SEVERITY_INFO     = 1
uint8 SEVERITY_NOTICE   = 2
uint8 SEVERITY_WARN     = 3
uint8 SEVERITY_ERROR    = 4
uint8 SEVERITY_CRITICAL = 5
uint8 SEVERITY_ALERT    = 6
uint8 SEVERITY_EMERG    = 7

builtin_interfaces/Time timestamp  # Time when this fault occurred or was updated
string name                        # Human-readable fault name
uint32 code                        # Product-defined fault code
uint8 type                         # Event type; severity changes use TYPE_START
uint8[] severities                 # Severity levels associated with this fault

# Reserved for robot-side fault diagnostics.
# SDK applications should ignore these fields; their contents and semantics are not part of the public interface.
string[] resources
bool grouped
string[] source
string details
