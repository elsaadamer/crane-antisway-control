Crane Anti-Sway Control System
A real time anti sway system for a scaled-down tower crane, built as a research project at TU Clausthal.
A USB camera detects how far the hanging load has swung, and the system automatically drives the trolley motor to cancel the oscillation.

How It Works

A camera mounted on the crane crate looks down at an ArUco marker attached to the load
A Python script measures how far the marker has shifted from the center of the frame
That offset is sent via serial port to an Arduino Uno
The Arduino runs a proportional controller that drives the trolley motor in the opposite direction of the swing
A state machine manages the sequence: idle → joystick control → 2-second settling delay → anti-sway correction

Control Law
The trolley speed is proportional to the displacement error:
v = Kp × |x_marker − x_center|
Two gain values are used to avoid overshoot:

Kp = 5.0 for large displacements (fast initial correction)
Kp = 2.5 for small displacements (gentle final alignment)

Hardware
PartDetailsMicrocontrollerArduino Uno (ATmega328P)Motor driversL298N dual H-bridge × 2Motors9V DC, ~200 RPM, 1:48 gear ratioCameraELP 1080P USB (up to 100 fps)ArUco markerDICT_4X4_50, 100mm printed, attached to load

Software

Python — OpenCV, cv2.aruco, PySerial, NumPy, Matplotlib
Arduino / C — State machine, P-controller, PWM motor control, serial parser with start/end markers

File Structure
├── arduino/
│   └── crane_antisway_controller.ino
├── python/
│   └── antisway_vision.py
├── docs/
│   └── research_paper.pdf
├── media/
│   └── (photos and videos)
└── README.md

Results
Oscillation amplitude was reduced significantly compared to natural damping alone.
Full measurement graphs are in docs/research_paper.pdf.

Academic Context
Research project — TU Clausthal, Mechatronics Department
Supervisor: Mykhailo Kutia | Professor: Stefan Palis
