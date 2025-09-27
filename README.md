## Laser coolant (MODIFIED)

Under development. Adds monitoring for \(tube\) coolant controlled by `M8`, configurable by settings.

* `$378` - time in seconds after coolant is turned on before an alarm is raised if the coolant ok signal is not asserted.
* `$379` - time in minutes after program end before coolant is turned off. \(WIP\)
* `$380` - min coolant temperature allowed. \(WIP\)
* `$381` - max coolant temperature allowed. \(WIP\)
* `$382` - input value offset for temperature calculation. \(WIP\)
* `$383` - input value gain factor for temperature calculation. \(WIP\)

WIP - Work In Progress.

Dependencies:

Driver must have at least one [ioports port](../../templates/ioports.c) input available for the coolant ok signal.
An optional analog input port is required for coolant temperature monitoring.

---
2025-09-24
