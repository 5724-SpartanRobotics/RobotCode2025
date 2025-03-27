package frc.robot.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDFfController extends PIDController {
    private double m_kff;

    /**
     * @param kp proportion
     * @param ki integral
     * @param kd derivative
     * @param kff feedforward
     * @throws IllegalArgumentException if Kff &lt; 0
     */
    public PIDFfController(double kp, double ki, double kd, double kff) {
        super(kp, ki, kd);
        this.m_kff = kff;
        if (kff < 0.0) {
            throw new IllegalArgumentException("Kff must be a non-negative number!");
        }
    }

    /**
     * @param kp proportion
     * @param ki integral
     * @param kd derivative
     * @param kff feedforward
     * @param period period between updates (seconds)
     * @throws IllegalArgumentException if Kff &lt; 0
     */
    public PIDFfController(double kp, double ki, double kd, double kff, double period) {
        super(kp, ki, kd, period);
        this.m_kff = kff;
        if (kff < 0.0) {
            throw new IllegalArgumentException("Kff must be a non-negative number!");
        }
    }

    public void setPIDFf(double kp, double ki, double kd, double kff) {
        this.setPID(kp, ki, kd);
        this.m_kff = kff;
    }

    public void setFf(double kff) {
        this.m_kff = kff;
    }

    /**
     * get feedforward
     * @return feedforward
     */
    public double getFf() {
        return m_kff;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("ff", this::getFf, this::setFf);
    }
}
