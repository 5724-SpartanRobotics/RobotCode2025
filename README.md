# Changelog
Changelog based on what we do, but some of it may have to come from git commit history.

## Feb 22, 2025
* Removed the calling of the code to set the PID setpoints at teleop start.
* Added code to call Stop on the PID ramps at disabled init.
* Added sequential sequences for more arm/elevator actions.
* Adjusted the algae out of the way for arm rotate setpoint
## Feb 21, 2025
* Alan adjusted LED to flash once on robot boot
* Applied PID ramp to 4 position regulators
* Fixed the swerve turn initial position reset off absolute encoder code.
## Feb 20, 2025
* Adjust more PIDs (algae and arm rotate)
* Adjust CAN IDs

## Feb 19, 2025
* Added in some Limelight 3G vision code... Will it work? Probably not. Will we use it? No clue, but it's there.
* Nothing crashes in simulation!

## Feb 18, 2025
* Tuned the PID for the elevator & the extending arm (i.e., the elevator goes up/down & the arm goes in/out)
* Changed the way some buttons on the Operator controller fire commands
* Fix some errors with position feedback scaling
* Added a method in Robot to get the Util.Color based on the current DS alliance.

## Feb 17, 2025
* Moved some constants from Interference class to Constants classes
* changed the operator control from XBox to Joystick - button mappings in Google Drive Spreadsheet under technical
* Added some methods that are not implemented for moving to L4 - L1. I think these need to be a sequence that keeps the robot from tearing itself up.
* Added a Hold Game piece position for Algae arm.

## Feb 16, 2025
* Added Interference helper for elevator/arm/intake
* Fixed a null "pointer" discovered while using simgui
* Build successful, more simulation to be done later
* We may replace operator Xbox controller with a Joystick

## Feb 15, 2025
### Part 1
* LED code works properly
* Some Algae intake added / worked on
* Add commands & button mappings for elevator, arm, and intake (lots of code)
### Part 2
* Add logging
* Add CameraServer
* Remove unread variables in SwerveModule
* Remove the DriveTrainInterface
* Move Constants out of Subsystems, since it's not a subsystem
* Move SwerveModule into new `lib` folder, since it is also not a subsystem
* Removed duplicate 2π in Util.Conversions
* Make some formatting changes

## Feb 14, 2025
* Add the TeleopSwerve default driving command
* Fix some non-instantiated things
* More work on the LED system, which don't work as intended
* Build Successfule
* Move some constants around
* Update CAN IDs
* some debugging (which was not removed before commit)

## Feb 12, 2025
* Add LED subsystem for an led "status" strip
* Added the claw motor to the Elevator & Arm subsystem
* Added a check for high current use on the claw, which activates green LEDs
* All of this still needs to be tested
* Build Successful

## Feb 11, 2025
### Part 1
* Import initial drivetrain code (25 errors, 2 warnings) - subsystem, interface, swerve modules
* Import some old code as well (would like to get rid of old stuff if possible)
* Install dependencies
* Build Failed
### Part 2
* Fix name errors (`Subsystems` -> `subsystems`)
* Remove unused imports
* Change return types for drive methods in drivetrain (now they return themselves)
* Almost completely reworked the drivetrain subsystem (got rid of the errors and warnings)
* Almost completely redid the Swerve Module (based on my previous code, 9 problems are unused fields + 1 is unused import)
* Build Successful
### Part 3
* I decided to add this changelog
* I added a symlink LICENSE -> WPILib-License.md
* Supressed a warning (unued field) & removed unused import.

## Feb 8, 2025
* Project started from scratch
* Map buttons
* Create Arm & Elevator subsystem