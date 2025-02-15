package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constant.LedConstants;

public class LedSubsystem extends SubsystemBase {
    public static final Color kWhiteWhiteGreen = Color.fromHSV(217, 73, 96);

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

    public LedSubsystem _SETCOLOR_UNSAFE(Color color) {
        for (int i = 0; i < _ledBuffer.getLength(); i++) {
            switch (i % 4) {
                case 0: _ledBuffer.setRGB(i, (int)(color.red * 255) % 255, (int)(color.green * 255) % 255, (int)(color.blue * 255) % 255); break;
                case 1: _ledBuffer.setRGB(i, (int)(color.green * 255) % 255, (int)(color.blue * 255) % 255, (int)(color.red * 255) % 255); break;
                case 2: _ledBuffer.setRGB(i, (int)(color.blue * 255) % 255, (int)(color.red * 255) % 255, (int)(color.green * 255) % 255); break;
                case 3: _ledBuffer.setRGB(i, 0, 0, 0); break;
                default: break;
            }
        }
        _led.setData(_ledBuffer);
        return this;
    }

    public LedSubsystem setGreen() {
        for (int i = 0; i < _ledBuffer.getLength(); i++) {
            switch (i % 4) {
                case 0: _ledBuffer.setRGB(i, 0, 255, 0); break;
                case 1: _ledBuffer.setRGB(i, 255, 0, 0); break;
                case 2: _ledBuffer.setRGB(i, 0, 0, 255); break;
                case 3: _ledBuffer.setRGB(i, 00, 00, 0); break;
                default: break;
            }
        }
        _led.setData(_ledBuffer);
        return this;
    }

    public LedSubsystem reset() {
        LEDPattern.solid(Color.kBlack).applyTo(_ledBuffer);
        _led.setData(_ledBuffer);
        return this;
    }

    public LedSubsystem off() {
        LEDPattern.solid(Color.kBlack).applyTo(_ledBuffer);
        _led.setData(_ledBuffer);
        return this;
    }
}
