# DeskBot 

https://casterba60.github.io/DeskBot/

This is a fully 3D-printed, 4-DOF robotic arm built for precision motion control and educational exploration of embedded systems and robotics. The system uses a hybrid actuation scheme combining geared DC motors for high-torque joints and MG996R servos for lightweight, responsive movements at the wrist and gripper. A custom-designed PCB centered around an STM32 microcontroller powers the control logic, offering real-time performance and rich peripheral support.

**Key Features**

Fully 3D-Printed Design: Constructed from PLA/PETG for rapid fabrication, modularity, and easy customization.

Custom STM32-Based Control PCB:

Integrated H-bridge drivers for controlling DC motors with PWM.

Servo headers with regulated power for MG996R units.

Cascading PID loop architecture for precise control of joint positions.

**4 Degrees of Freedom:**

Base, Shoulder, Elbow: Actuated with geared DC motors for strength and load-bearing stability.

Wrist Rotation: Powered by MG996R servo for precise articulation.

End Effector: Two-finger gripper actuated by an MG996R servo for basic object manipulation (not counted in DOF).

Cascading PID Control: Real-time feedback control for DC joints ensures smooth, accurate positioning.

Inverse Kinematics Support [coming soon]: Implements a lightweight IK solver for Cartesian end-effector positioning, enabling intuitive point-to-point motion planning.

Firmware: Written in C for the STM32 platform using STM32Cube or bare-metal HAL, enabling tight control loops and hardware-level optimizations.

This project serves as a versatile platform for robotics education, embedded firmware development, and motion control research.
