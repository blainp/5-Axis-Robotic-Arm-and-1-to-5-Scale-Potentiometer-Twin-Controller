# Custom-1-Meter-3D-Printed-Robotic-Arm-with-1:5-Miniature-Potentiometer-Based-Twin-Controller
A fully custom, 3D-printed robotic arm utilizing 4 NEMA 17 joints with 3D printed gearboxes in addition to 2 MG996R servo motors. Controlled via Arduino and a Python/tkinter interface utilizing a custom controller which is a scaled-down replica of the arm using potentiometers in place of motors (with a switch for claw actuation) as an intuitive physical input device.


<p align="center">
<img width="400" height="400" alt="ImageToStl com_ModelRobotArm step (1)" src="https://github.com/user-attachments/assets/d884b007-47e7-4f2a-899d-c51433145355" />
<img width="400" height="400" alt="ImageToStl com_potentiometer+knob+v2 step" src="https://github.com/user-attachments/assets/50e3c30f-8a51-46f6-9174-8a63ff34b130" />
</p>

<!--<table align="center" style="border-collapse: collapse; border: none;">
  <tr>
    <td align="center" style="border: none;">
      <img width="400" height="400" alt="Robot Arm Model" src="https://github.com/user-attachments/assets/d884b007-47e7-4f2a-899d-c51433145355" />
      <p><b>5-Axis Robot Arm</b></p>
    </td>
    <td align="center" style="border: none;">
      <img width="400" height="400" alt="Potentiometer Knob" src="https://github.com/user-attachments/assets/50e3c30f-8a51-46f6-9174-8a63ff34b130" />
      <p><b>Twin Controller</b></p>
    </td>
  </tr>
</table>
-->

## Project Overview

<p align="center">
<img width="400" height="500" alt="636255024-16cb6c80-8e17-448b-934a-791ced426ac2 (2)" src="https://github.com/user-attachments/assets/78ff26d0-3015-492f-9985-d7cacdf6fda5" />
<img width="400" height="500" alt="636259470-e3cbd5e3-f8db-4ea4-9059-88ec55b0ac9d (3)" src="https://github.com/user-attachments/assets/d2de7196-6e24-484a-9688-bf4670c581c0" />




</p>

This project is a fully custom 1-meter robotic arm, 4 stepper-driven joints plus a servo wrist and claw. It is entirely 3D printed, including the gearboxes, except for 1 metal 71:1 planetary gearbox utilized on the lowest shoulder joint due to torque constraints and gear slippage. It was designed in Fusion 360 as a complete assembly first, then refined during the build, arm lengths, screw holes, and mounting points all went through minor revisions as issues came up on the bench.

<p align="center">
  <video src="Media/ArmDemonstrationVid.mp4" width="800" controls autoplay muted loop playsinline>
    Your browser doesn't support embedded video. <a href="Media/ArmDemonstrationVid.mp4">Download the demo</a>.
  </video>
</p>

### Hardware & Power Architecture

The arm uses 4 NEMA 17 stepper motors (base, shoulder1, shoulder2, elbow) driving custom 3D printed planetary gearboxes, plus an MG996R servo for wrist rotation and a second servo for claw actuation.

**Gearboxes:** Three of the four upper joints use a modified version of [this 80:1 NEMA 17 planetary gearbox](https://www.printables.com/model/202734-801-nema-17-planetary-gearbox/comments), adjusted for the shaft length of the motors used here. Cycloidal designs were also considered, but with limited access to the hardware needed to machine/print those reliably at the time, the planetary route was used instead for its relative simplicity.

**Base Elbow Joint:** The 3D printed gearbox held up fine on the upper joints, but skipped under the torque loading at the base elbow when fully extended, since it carries the weight of everything above it. The base motor was swapped to an off-the-shelf [71:1 metal planetary gearbox, H-type shaft](https://www.aliexpress.com/item/1005006217641964.html) instead.

**Rotating Base:** To allow the arm to rotate around its base ended up being a notable challenge, as the specific gearbox design I utilized did not perform well with a direct load, which presses directly into the gearbox assembly, being placed on the output shaft. This meant I couldn't simply place the arm directly on-top of a gearbox, and had to find a way to transfer the majority of the arm's weight into the base and not through the gearbox itself. This was achieved by modelling two circular, semi-cylindric channels, one into the bottom of the arms bottom plate, which attached to the base motor, and one into the top of the base's top plate, which surrounded the output shaft of the gearbox. In the channel 6mm metal ball bearings were placed to allow the arm to rotate by rolling on them, akin to a complete bearing assembly. This allowed the arm to transfer the majority of its weight into the base of the system whilst putting very little vertical pressure on the gearbox assembly. The gearbox was then attached to the base through a hole in the center of the plate which was surrounded by the bearing channel.

**Claw:** The gripper itself is a modified version of [this flexible MG995/MG996R servo gripper](https://cults3d.com/en/3d-model/various/robot-gripper-flexible-servo-mg995-mg996r), adapted to mount onto the wrist assembly.

<p align="center">
<img width="800" alt="Schematic_New-Project_2026-08-16 (1)" src="https://github.com/user-attachments/assets/506e4406-2a06-4dfe-85a4-60b76ff7470b" />
</p>

### Control Scheme

Keyboard control was tried first, but with 5 independent axes it was hard to track which key drove which joint. The current setup uses a 1:5 scale physical twin of the arm as the input device with potentiometers standing in for each motor, and a switch on the mini controller end which handles the claw's open/closed state.

The system runs across two Arduinos and one Python/tkinter app:

- **Arm Arduino** runs [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) for the 5 stepper channels and [ServoTimer2](https://github.com/nabontra/ServoTimer2) for the wrist/claw servos (ServoTimer2 specifically to avoid clobbering AccelStepper's use of Timer1). It parses `<motor0,motor1,motor2,motor3,motor4,wrist,claw>` commands over serial, ramps stepper speed smoothly toward the commanded direction, and reports back live joint angles every 100ms.
- **Potentiometer control-arm Arduino** reads 5 pots (4 joints + a dedicated wrist pot) plus the claw switch, and streams them out over its own serial port.
- **Python/tkinter app (`ArmLooky.py`)** is the master controller. It runs two background serial threads (one per Arduino), smooths incoming pot readings (median filter + EMA), and drives the arm in one of two modes:
  - **Manual mode**, press-and-hold +/- buttons per joint, useful for fine positioning or testing without the control arm connected.
  - **Potentiometer mode**, the mini twin arm drives joint targets directly, with position deltas rate-limited so the real arm can't be commanded to jump instantly.

  Wrist rotation is always live from its dedicated pot regardless of mode, since it's a direct analog pass-through rather than a button-driven target. Because the wrist pot's electrical sweep is wider than the servo's usable range, its raw reading is linearly rescaled onto 0-180° instead of being clamped, clamping was the original bug that made the wrist appear to stop responding past a certain point.

  Both the real arm and the potentiometer arm expose 5 physical channels, but only 4 correspond to real stepper joints. A `MAP`/`MOTOR_MULT` pair translates between UI joint order and physical stepper index, and the unused 5th physical slot is simply never driven.

<p align="center">
<img width="700" alt="PLACEHOLDER - control loop / serial architecture diagram" src="" />
</p>

**Note on iteration:** An earlier version of the firmware treated the wrist command the same way as the stepper directions (a `-1/0/1` value), which silently discarded every real position value sent from Python, so the wrist pot could be turned all day and nothing would happen. Fixing this meant switching the wrist to an absolute 0-180° position command instead of a relative direction.

---

## Features

### Fully Custom Arm
- 4 NEMA 17 stepper joints plus servo-driven wrist rotation and claw, entirely 3D printed, including the gearboxes, except for the base gearbox.
- Custom 80:1 planetary gearboxes on the upper joints, metal 71:1 gearbox on the base after torque-related issues with the printed version.

### Closed-Loop Joint Control
- Real arm reports live joint angles back over serial, target vs. actual position drives motor direction until within tolerance.
- Smoothed stepper acceleration rather than instant direction snaps, reducing mechanical shock on the gearboxes.

### Physical Twin Controller
- 1:5 scale replica of the full arm, using potentiometers in place of each motor as direct analog input.
- Switch-based claw actuation on the mini controller.
- Toggleable manual button mode for fine positioning without the control arm connected.

---

## Repository Structure

```text
/Diagrams
    Wiring diagrams and control-loop / serial architecture diagrams

/Documentation
    Design notes, BOM, and iteration history

/Firmware
    Arduino code for both the arm and the potentiometer control-arm

/Software
    Python/tkinter master controller (ArmLooky.py) and control-loop utilities

/3D Models
    Fusion 360 files and STLs for the arm, gearboxes, and mini control-arm
```

---

## Bill of Materials (BOM)

| Component | Description | Function |
| :--- | :--- | :--- |
| Microcontroller (Arm) | Arduino (AccelStepper + ServoTimer2) | Drives 5 stepper channels + wrist/claw servos, reports joint angles |
| Microcontroller (Control Arm) | Arduino | Reads 5 potentiometers + claw switch, streams over serial |
| Joint Motors | 5x NEMA 17 Stepper Motors | Base, Shoulder, Aux, Elbow, + 1 unused physical channel |
| Upper Gearboxes | Modified 80:1 3D Printed Planetary Gearbox (x4) | Torque reduction for upper joints |
| Base Gearbox | 71:1 Metal Planetary Gearbox, H-type shaft | Torque reduction for the base joint (printed gearbox skipped under load here) |
| Wrist Actuator | MG996R Servo | Wrist rotation, absolute 0-180° position control |
| Claw Actuator | Servo (modified gripper design) | Claw open/close |
| PLACEHOLDER | | |

<p align="center">
<img width="1000" alt="PLACEHOLDER - full BOM table / parts layout photo" src="" />
</p>

---

## Known Constraints & Future Recommendations

### Base Joint Torque
The original all-printed gearbox design skips under sustained load on the base joint, since it carries the weight of the entire arm above it. The metal 71:1 gearbox swap resolved this, but a stiffer printed design (or metal gearboxes across all joints) remains worth exploring if weight/cost allows.

### Potentiometer Noise
Raw pot readings are noisy enough on their own that both a median filter and an EMA low-pass are needed before the values are usable as targets, worth keeping in mind for anyone wiring their own control-arm pots, especially over longer leads.

### Motor Location Reporting
Currently, due to the nature of NEMA17 stepper motors, their absolute positioning in relation to their current angle, which is sent between the arduino and python programs, is prone to error. The lack of a rotary encoder on the output of such gearboxes means that the NEMA17s rotations are simply divided by the 80:1 figure of the gearboxes' output. These modified figures are then reported to the python program, and there is no verification or error handling of motor positions in the event of a skipped gearbox. Although it is very rare that any of the current gearboxes skip when exclusively under the weight of the arm, accidental contact with a foreign body, or the ground, can cause such skipping, and mis-align the virtual and physical angles of a motor. Therefore, perhaps the most important future improvement to such a system would be rotary encoders on gearbox outputs to verify and report angles of rotation more accurately.
