I independently designed, coded, wired, and built a fully holonomic swerve drive robot from the ground up. Unlike traditional coaxial swerve systems that rely on complex bevel gear assemblies, my design uses compact in-wheel (hub) motors, eliminating the need for costly mechanical gearing. This approach greatly simplifies the drivetrain while maintaining precise, smooth omnidirectional control.

From CAD design and prototyping to electronics integration, control software, and testing, every stage of the project was developed through my own iteration and problem-solving. My goal throughout has been to create a system that is functional, efficient, reliable, and elegant in its simplicity.

I designed the entire drivetrain in Fusion 360, modeling over 20 custom parts with tight tolerances and seamless mechanical integration. Every component fits together compactly and efficiently, resulting in a highly optimized and manufacturable system. The CAD work alone represents hundreds of hours of design iteration and mechanical refinement.

Throughout development, I produced multiple design iterations and continuous improvements, including a major redesign that replaced a bulky external encoder setup with a fully internal encoder system, dramatically reducing size, complexity, and environmental exposure while improving overall reliability.

Each wheel module integrates one drive motor, one steering motor, and a fully contactless encoder for precise position feedback and smooth, accurate control. Capable of translating in any direction (forward, backward, sideways, and diagonally) while also rotating in place or turning dynamically during motion. Every module component, including the gears, is designed for additive manufacturing. Only standard hardware (bearings, bolts, and motors) are required, making the system highly accessible and easy to reproduce. All four modules share a symmetrical design and mount to a standard 1" x 1" aluminum extrusion frame, enabling adjustable frame sizes and simplifying assembly. Each module is fully enclosed to protect gears and internal components from impacts and debris, improving longevity and reliability. Uses readily available 555 brushed motors, balancing performance, affordability, and ease of maintenance.


The swerve drive system is powered by a Teensy 4.1 microcontroller, chosen for its high clock speed, abundant I/O pins, and real-time processing capabilities: essential for precise control of 8 motors and 4 encoders. Wiring such a complex system required extensive planning and iteration to ensure reliability, maintainability, and performance.


Meticulous attention to wiring layout was critical throughout the design. Keeping logic-level signals separated from power lines prevented electromagnetic interference, and maintaining clean, organized wiring made iterative redesigns and troubleshooting manageable. These refinements, developed over several months, significantly improved system reliability, precision, and ease of maintenance.


Developing the software for this project required significant learning and problem-solving. The system is written entirely in C++, totaling over 500 lines of code that manage multiple complex tasks simultaneously: Running 4 independent PID loops for precise steering control of brushed motors, reading 4 encoder angles sequentially via SPI to provide real-time feedback, processing 3 joystick channels via PWM interrupts for translation and rotation commands, executing full inverse kinematics, converting joystick inputs and optional robot heading into precise motor angles and speeds for all four modules.


Since the steering motors use standard brushed motors instead of servos, open-loop control was insufficient. By integrating encoder feedback into closed-loop PID control, each motor continuously calculates the error (the difference between desired and actual position) and applies proportional, integral, and derivative corrections (PID), carefully tuned through extensive testing. This allows the robot to achieve ultra-smooth, tight, and responsive motion.
The control loop runs at approximately 75Hz, providing near-real-time execution with minimal lag. Additional software features include:
Robot-oriented vs. field-oriented (headless) driving, selectable on the fly.
Shortest-path steering optimizations, including automatic 180° inversion of modules to minimize rotation and maximize responsiveness.
Numerous other code-level optimizations for efficiency, precision, and reliability.

Developing this system required hundreds of hours of testing, iteration, and fine-tuning, combining electronics, software, and mechanical integration into a fully functional, high-performance swerve drive.

<img width="1323" height="912" alt="image" src="https://github.com/user-attachments/assets/5500b9fa-73fc-4c83-b24e-708715f1c0af" />

<img width="1248" height="969" alt="image" src="https://github.com/user-attachments/assets/40575971-6605-4b11-9345-e2367205ad56" />
