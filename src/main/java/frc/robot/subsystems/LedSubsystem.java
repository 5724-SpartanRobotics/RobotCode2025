package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constant.LedConstants;

public class LedSubsystem extends SubsystemBase {
    public static final Color kWhiteWhiteGreen = Color.fromHSV(217, 73, 96);
    public static int percentTo255(double percent) { return Math.min((int)(percent * 255), 255); }
    public static final class RGBW {
        public static final int R = 0;
        public static final int G = 0;
        public static final int B = 0;
        public static final int W = 0;
    }

    private final AddressableLED _led;
    private final AddressableLEDBuffer _ledBuffer;

    public LedSubsystem() {
        this._led = new AddressableLED(LedConstants.port);
        // The actual color order is GRBW, but we have it set to RGB so it's easier to visualize the
        // color order in the code below.
        this._led.setColorOrder(ColorOrder.kRGB);
        this._ledBuffer = new AddressableLEDBuffer(LedConstants.stripLength);
        this._led.setLength(this._ledBuffer.getLength());
        this._led.setData(this._ledBuffer);
        this._led.start();
    }

    @SuppressWarnings("unused")
    private LedSubsystem setColor__UNSAFE(Color color) {
        LEDPattern lp = LEDPattern.solid(color);
        lp.applyTo(_ledBuffer);
        _led.setData(_ledBuffer);
        return this;
    }

    @SuppressWarnings("unused")
    private LedSubsystem _SETCOLOR_UNSAFE_(Color color) {
        for (int i = 0; i < _ledBuffer.getLength(); i++) {
            switch (i % 4) {
                case 0: _ledBuffer.setRGB(i, percentTo255(color.red), percentTo255(color.green), percentTo255(color.blue)); break;
                case 1: _ledBuffer.setRGB(i, percentTo255(color.green), percentTo255(color.blue), percentTo255(color.red)); break;
                case 2: _ledBuffer.setRGB(i, percentTo255(color.blue), percentTo255(color.red), percentTo255(color.green)); break;
                case 3: _ledBuffer.setRGB(i, 0, 0, 0); break;
                default: break;
            }
        }
        _led.setData(_ledBuffer);
        return this;
    }

    public LedSubsystem setColor(Color color) {
        for (int i = 0; i < _ledBuffer.getLength(); i++) {
            switch (i % 4) {
                // This says "setRGB", but we're just using it as a hack to set the bytes in a specific order.
                // The LED strip we have has color order GRBW, so that's why for the first component we pass
                // green, then the second X, etc., etc.
                case 0: _ledBuffer.setRGB(i, percentTo255(color.green), percentTo255(color.red), percentTo255(color.blue)); break;
                case 1: _ledBuffer.setRGB(i, RGBW.W, percentTo255(color.green), percentTo255(color.red)); break;
                case 2: _ledBuffer.setRGB(i, percentTo255(color.blue), RGBW.W, percentTo255(color.green)); break;
                case 3: _ledBuffer.setRGB(i, percentTo255(color.red), percentTo255(color.blue), RGBW.W); break;
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
        setColor(Color.kBlack);
        return this;
    }

    public LedSubsystem off() {
        LEDPattern.solid(Color.kBlack).applyTo(_ledBuffer);
        _led.setData(_ledBuffer);
        return this;
    }
}
