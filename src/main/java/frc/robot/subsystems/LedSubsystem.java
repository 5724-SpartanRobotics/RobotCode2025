package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constant.LedConstants;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED _led;
    private final AddressableLEDBuffer _ledBuffer;

    public LedSubsystem() {
        this._led = new AddressableLED(LedConstants.port);
        this._ledBuffer = new AddressableLEDBuffer(LedConstants.stripLength);
        this._led.setLength(this._ledBuffer.getLength());
        this._led.setData(this._ledBuffer);
        this._led.start();
    }

    public LedSubsystem setColor(Color color) {
        LEDPattern lp = LEDPattern.solid(color);
        lp.applyTo(_ledBuffer);
        _led.setData(_ledBuffer);
        return this;
    }

    public LedSubsystem off() {
        LEDPattern.solid(Color.kBlack).applyTo(_ledBuffer);
        _led.setData(_ledBuffer);
        return this;
    }
}
