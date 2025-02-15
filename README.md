# Changelog
Changelog based on what we do, but some of it may have to come from git commit history.

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