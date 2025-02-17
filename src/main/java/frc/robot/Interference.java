package frc.robot;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorAndArmSubSys;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorAndArmConstants;

/*Provides indications of interference between different mechanisms
 */
public class Interference {

    private ElevatorAndArmSubSys _EleArm;
    private AlgaeSubsystem _Algae;
    public Interference(AlgaeSubsystem algae, ElevatorAndArmSubSys elevatorAndArm) {
        _EleArm = elevatorAndArm;
        _Algae = algae;
    }

    public Boolean ArmCannotRotateDown(InterferenceInfo info)
    {
        double currentArmRotate = _EleArm.GetArmRotateAngleDegrees();
        double currentAlgaeAngle = _Algae.GetAlgaeArmAngleDegrees();
        double currentArmExtend = _EleArm.GetArmExtendPosnInches();
        double currentEleHeight = _EleArm.GetElevatorHeightInches();

        if (currentAlgaeAngle <= AlgaeConstants.AlgaeMinAngleForArmRotationClearance && currentArmRotate < ElevatorAndArmConstants.ArmAlgaeZoneAngle)
        {
            info.Message = "Algae arm is blocking main arm rotate down";
        }
        else if ((currentArmExtend - currentEleHeight) > ElevatorAndArmConstants.ArmExtendMaxWhenInRobot && currentArmRotate <= ElevatorAndArmConstants.ArmAlgaeZoneAngle)
        {
            info.Message = "The arm is extended too far which is blocking arm rotation down";
            return true;
        }
        //don't allow rotation down if our claw is outside of our max parimeter. We only allow retraction
        else if (currentArmRotate > 90 && _EleArm.GetArmOutsideFrameInches() > ElevatorAndArmConstants.ArmMaxOutsideOfFrameInches)
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
        double currentAlgaeAngle = _Algae.GetAlgaeArmAngleDegrees();
        double currentArmExtend = _EleArm.GetArmExtendPosnInches();
        double currentEleHeight = _EleArm.GetElevatorHeightInches();

        if (currentAlgaeAngle <= AlgaeConstants.AlgaeMinAngleForArmRotationClearance && currentArmRotate >= ElevatorAndArmConstants.ArmAlgaeZoneAngle)
        {
            info.Message = "Algae arm is blocking main arm rotation up";
            return true;
        }
        else if ((currentArmExtend - currentEleHeight) > ElevatorAndArmConstants.ArmExtendMaxWhenInRobot && currentArmRotate <= ElevatorAndArmConstants.ArmAlgaeZoneAngle)
        {
            info.Message = "The arm is extended too far which is blocking arm rotation out";
            return true;
        }
        //don't allow rotation up if our claw is outside of our max parimeter. We only allow retraction
        else if (currentArmRotate <= 90 && _EleArm.GetArmOutsideFrameInches() > ElevatorAndArmConstants.ArmMaxOutsideOfFrameInches)
        {
            info.Message = "The arm rotate up is not allowed, it is almost out of frame. Please retract";
            return true;
        }
        info.Message = null;
        return false;
    }
}
