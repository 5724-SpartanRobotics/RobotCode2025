package frc.robot;

import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorAndArmSubSys;

/*Provides indications of interference between different mechanisms
 */
public class Interference {

    private ElevatorAndArmSubSys _EleArm;
    // private AlgaeSubsystem _Algae;
    public Interference(AlgaeSubsystem algae, ElevatorAndArmSubSys elevatorAndArm) {
        _EleArm = elevatorAndArm;
        // _Algae = algae;
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
