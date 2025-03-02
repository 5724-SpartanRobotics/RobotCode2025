package frc.robot;

import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.subsystems.ArmSubSys;

/*Provides indications of interference between different mechanisms
 */
public class Interference {

    private ArmSubSys _EleArm;
    public Interference(ArmSubSys elevatorAndArm) {
        _EleArm = elevatorAndArm;
    }

    public Boolean ArmCannotRotateDown(InterferenceInfo info)
    {
        double currentArmRotate = _EleArm.GetArmRotateAngleDegrees();
    
        //don't allow rotation down if our claw is outside of our max parimeter. We only allow retraction
        if (currentArmRotate > 90 && _EleArm.GetArmOutsideFrameInches() > ElevatorAndArmConstants.ArmMaxOutsideOfFrameInches)
        {
            info.Message = "The arm rotate down is not allowed, it is almost out of frame. Please retract";
            return true;
        }
        info.Message = null;
        return false;
    }

    public Boolean ArmCannotRotateUp(InterferenceInfo info)
    {
        double currentArmRotate = _EleArm.GetArmRotateAngleDegrees();

        //don't allow rotation up if our claw is outside of our max parimeter. We only allow retraction
        if (currentArmRotate <= 90 && _EleArm.GetArmOutsideFrameInches() > ElevatorAndArmConstants.ArmMaxOutsideOfFrameInches)
        {
            info.Message = "The arm rotate up is not allowed, it is almost out of frame. Please retract";
            return true;
        }
        info.Message = null;
        return false;
    }
}
