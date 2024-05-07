HyperDroids 2023/2024
=====================

This is Hyperdroid's 2023/2024 robot code.

Please put notes in this README.


Building and Running
--------------------

- Install Android Studio "Giraffe" as per FTC instructions.
- Clone this repository (New -> Project from Version Control)

- Build the code once while connected to the "real" internet
- Pair with the "10015-RC" wifi direct network (password on the driver hub)
  - Test connection: visit http://192.168.43.1:8080 and you should see something
- Locally, run "adb connect 192.168.43.1:5555". This should succeed.
  - On one linux system this is: ./Android/Sdk/platform-tools/adb connect  192.168.43.1:5555
- In "device manager" in Android Studio, you should find a "REV Robotics Control Hub v1.0" device in the "Physical" section
   - Recommend: delete all other virtual and physical devices if you don't do other Java work
- Should be able to push a build to the robot (press the green "play" button)


General Philosophy and Structure
--------------------------------

We follow a more "event-based" style of thinking with this code.

We attempt to share as much code as possible between Autonomous and TeleOp modes.

Logical subsystems of the robot exist as a Java class (but not _everything_ on the robot needs to have a class).

We make use of the following libraries and tools:

- FTCLib: https://ftclib.org/
- FTC-Dashboard: https://acmerobotics.github.io/ftc-dashboard/
- EasyOpenCV: https://github.com/OpenFTC/EasyOpenCV/
- EasyOpenCV-Sim: https://deltacv.gitbook.io/eocv-sim
- blkbx: https://github.com/meejah/blckbx


Overview of the Code
--------------------

