# Changelog
Changelog based on what we do, but some of it may have to come from git commit history.

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