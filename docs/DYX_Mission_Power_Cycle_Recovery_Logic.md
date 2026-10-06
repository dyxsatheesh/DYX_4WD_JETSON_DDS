# DYX Rover Mission Power-Cycle Recovery Logic

## Goal

If the rover is powered OFF while a mission is already started and not finished, the rover must remember the mission progress.

After power ON, the rover must ask the operator:

- **Resume Mission**
- **Stop & Upload New Mission**

The rover must **never automatically start moving after reboot**.

---

## Normal Mission

```text
Upload Mission
    ↓
Prepare Trajectory
    ↓
Start Mission
    ↓
Mission RUNNING
```

While the mission is running, the backend continuously saves the current mission progress.

Saved information includes:

```text
Mission ID
Mission Run ID
Current Point
Completed Points
Skipped Points
Failed Points
Point Status
Mission Progress
Execution Mode
Marking Tool
```

This is stored in:

```text
mission_runtime.json
```

---

## Example Before Power OFF

```text
P1  COMPLETED
P2  COMPLETED
P3  COMPLETED
P4  COMPLETED
P5  COMPLETED
P6  COMPLETED
P7  CURRENT
P8  PENDING
P9  PENDING
```

If the rover is powered OFF while going to P7, the saved mission progress remains on the Jetson.

---

## After Rover Power ON

```text
Rover Power ON
    ↓
PX4 Boots
    ↓
Jetson Boots
    ↓
Backend Starts
    ↓
Backend Finds Existing Mission
    ↓
Backend Finds Saved Mission Runtime
    ↓
Checks That Mission Was Started and Not Finished
    ↓
Recovery Required
```

The rover must remain:

```text
Mission Motion OFF
No Automatic ARM
No Automatic Resume
No Automatic Movement
```

---

## Trajectory After Reboot

The old local trajectory must not be used directly because PX4 can create a new local origin after reboot.

So the backend automatically rebuilds the same mission trajectory.

```text
Same mission.csv
    ↓
Trajectory Generator
    ↓
Generate trajectory using current PX4 local origin
    ↓
Mission Manager receives the new trajectory
```

The operator does not need to press Prepare again.

---

## Restore Mission Progress

After the trajectory is ready, the backend restores the old mission progress.

Example:

```text
Saved Progress:

P1-P6 = COMPLETED
P7 = CURRENT
P8-P9 = PENDING
```

Recovery becomes:

```text
P1-P6 = COMPLETED
P7 = CURRENT
P8-P9 = PENDING
```

The mission must continue from the **first unresolved point**.

So:

```text
If P7 is not completed
→ Resume from P7

If P7 was already completed
→ Resume from P8
```

The rover must never restart from P1.

---

## Mission State After Recovery

Even if the rover was RUNNING before power OFF:

```text
Before Power OFF:
RUNNING
```

after reboot it must become:

```text
PAUSED
```

Final recovery state:

```text
Mission = PAUSED
Recovery Available = TRUE
Current Point = Saved Unfinished Point
```

The rover waits for the operator.

---

# Operator Options

## Option 1 — Resume Mission

```text
Operator presses Resume Mission
    ↓
Use Existing Normal Resume Logic
    ↓
Normal Mission Safety Checks Run
    ↓
PX4 OFFBOARD
    ↓
ARM
    ↓
Mission RUNNING
    ↓
Continue From Saved Current Point
```

We do not create a separate recovery resume logic.

The existing normal mission Resume function is used.

---

## Option 2 — Stop & Upload New Mission

```text
Operator presses Stop & Upload New Mission
    ↓
Stop Old Mission
    ↓
Clear Old Runtime Progress
    ↓
Clear Old Recovery State
    ↓
Return to Mission Upload
    ↓
Upload New Mission
    ↓
Normal Prepare
    ↓
Normal Start
```

---

# Battery Critical Case

Battery critical uses the same recovery system.

Example:

```text
Battery <= 23.80 V
    ↓
Critical condition confirmed
    ↓
10%
    ↓
Mission PAUSED
    ↓
Mission progress saved
```

If the rover is powered OFF after this:

```text
Power OFF
    ↓
Power ON
    ↓
Restore Mission
    ↓
PAUSED
    ↓
Resume Mission
OR
Stop & Upload New Mission
```

There is no separate battery recovery system.

---

# When Recovery Popup Should Appear

Recovery is required when:

```text
Mission exists
+
Mission was already started
+
Mission is not completed
+
Saved runtime exists
+
Rover rebooted
```

Typical previous states:

```text
RUNNING
PAUSED
WAITING_FOR_NEXT
```

---

# When Recovery Should NOT Appear

```text
Mission only uploaded but never started
Mission completed normally
Mission already stopped and cleared
No valid saved mission runtime
```

---

# Important Safety Rule

```text
POWER ON
    ↓
Restore Mission
    ↓
Restore Progress
    ↓
Mission PAUSED
    ↓
WAIT FOR OPERATOR
```

The rover must never automatically start driving after reboot.

Only the operator can choose:

```text
RESUME MISSION

or

STOP & UPLOAD NEW MISSION
```

---

# Final Architecture

```text
Mission Manager
    ↓
Current Mission Progress
    ↓
ROS Bridge
    ↓
mission_runtime.json
    ↓
POWER OFF
    ↓
POWER ON
    ↓
Backend Detects Unfinished Mission
    ↓
Automatically Rebuild Same Trajectory
    ↓
Restore Completed / Current Point Progress
    ↓
Mission Manager = PAUSED
    ↓
Frontend Popup
    ↓
┌──────────────────────────┐
│ Resume Mission           │
│                          │
│ Stop & Upload New Mission│
└──────────────────────────┘
```
