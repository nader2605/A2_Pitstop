# Controller E-stop build

This build uses the first detected USB game controller as the simulation E-stop input, following the pygame joystick pattern used in Lab 11. **Controller button 0** is the E-stop by default. Change `Controller_estop_button` near the top of `pitstop.py` if a different button is required.


---

# A2-Assessment-Pit-Stop-Robots

Run the integrated demo from this folder:

```powershell
python pitstop.py
```

Collision demo from the initial scene: **Obstacle IN → START**.

After an E-stop: **RESET → START** acknowledges the stop, then retraces the executed motion and wheel/nut attachment changes. RESET itself moves nothing. Recovery can be stopped again. If a new obstruction blocks recovery, remove it, then RESET → START to continue. When recovery reaches READY, press START again for a new pit stop. After a completed cycle, use the same RESET → START recovery before repeating it.

The Stäubli is collision-checked with the Lab 5 Question 2 method (`collision.py`): its links are line segments tested against triangle-mesh boxes for the demo obstacle and the deposited old tyre, plus a line-plane test against the floor. On a predicted hit it stops before that step (stop on contact). The new wheel is carried through a high outboard Cartesian clearance route before the final hub approach, with a conservative carried-wheel clearance check against the car body.

The robot model is `Staubli/Staubli.py`, class `Staubli`. `PitStopEnv.py` has only been updated to use the new Staubli names. No compatibility file is needed.

    20.09.2026
        Estasblished what robots we will be using (Nick, Shlok and Lucas)
        Establihed how we are going to tackle assessment
            each robot will have a seperate file, then loaded into a simulated environment

        To-Do
            contact tutor about real robots and change of selection for robots
            find car, tyre and nut models on github
            alter scope
                rather than a race car style pit stop, we have chosen to do a tyre change

    21.09.2026
        Re-evaluated overall task goal
        changed final goal to accomodate for task requirements

    27.09.2026
        Reselected chosen robots to accomodate for new task requirements

    11.10.2026
        Once again reselected chosen robots
        Lucas has added to Lucas_robot.py
            Now existing working swift implementation - need to go further with this and implement whole robot
    
    16.10.2026
        Added GUI sidebar onto ABB_irb140 (Shlok's robot)

## Current E-stop control

The simulation uses a USB game controller through `pygame` as the software E-stop input. The controller input is read using the same joystick pattern used in the Lab 11 examples. Releasing the controller button does not resume the task; use the GUI RESET control to return the simulation to stage 0.

## Controller button mapping

Confirmed Nintendo Switch Pro Controller mapping:

- `0` = A -> **START AUTO**
- `1` = B -> unused
- `2` = X -> unused
- `3` = Y -> unused
- `4` = Select -> **RESET to stage 0**
- `6` = Start -> **E-STOP**
- `9` = L1 -> unused
- `10` = R1 -> unused

Normal controller workflow:

1. Press **A** to start the automatic pit-stop sequence.
2. Press **Start** at any time during robot motion to latch the E-stop.
3. Release **Start**. Motion does not resume.
4. Press **Select** to reset the simulation to stage 0.
5. Press **A** separately to start a new cycle.

The Swift GUI controls remain available as well.


## Controller mapping (updated)
- Start (6): start automatic cycle from READY.
- B (1): latched simulation E-stop. Release does not restart.
- A (0): retrace recorded reverse path after E-stop and B release. Collision and E-stop checks remain active.
- X (2): request full stage-0 reset at any time, applied after current trajectory unwinds. Does not auto-start. Avoid during normal operation.
- Gamepad B is not an industrial safety-rated emergency-stop device.


## UR3e pit-stop marshal (integrated)
The UR3e lollipop marshal from the team project is included without replacing the existing KUKA, ABB or Stäubli models. It extends the pit-stop sign before the lift begins, and retracts it before car lowering. The sign, body and pedestal are rendered in Swift. The robot has an independent conservative approximate path-clearance check against the car and other arms, and each movement step calls the existing safety guard so B E-stop and X reset interrupt it. A-button recorded reverse recovery and full reset include its joint configuration. The UR3e is a custom geometry/kinematics visual model, not a safety-certified model. Full simulation/hardware execution remains to be tested in the target UTS environment.
