package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Conversions {
    public static double angle2rotations(Angle angle, double gearRatio) {
        return angle.in(Units.Rotations) * gearRatio;
    }

    public static double angle2rotations(Angle angle, double gearRatio, double countsPerRev) {
        return angle2rotations(angle, gearRatio) * countsPerRev;
    }

    public static double angularV2rotations(Measure<AngularVelocityUnit> angularVelocity, double gearRatio) {
        return angularVelocity.in(Units.RotationsPerSecond) * gearRatio;
    }

    public static double angularV2rotations(Measure<AngularVelocityUnit> angularVelocity, double gearRatio, double countsPerRev) {
        return angularV2rotations(angularVelocity, gearRatio) * countsPerRev;
    }

    public static double clampAngle(Angle value, Angle low, Angle high, AngleUnit unit) {
        return MathUtil.clamp(value.in(unit), low.in(unit), high.in(unit));
    }

    public static double clampDistance(Distance value, Distance low, Distance high, DistanceUnit unit) {
        return MathUtil.clamp(value.in(unit), low.in(unit), high.in(unit));
    }
}
