package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {
    /** you can set default colors to whatever you want. */
    public static Color kDefaultActiveColor = Color.kGreen;
    public static Color kDefaultInactiveColor = Color.kBlack;
    public static Color kDefaultNotificationColor = Color.kBeige;

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
    private final Timer _timer = new Timer();
    private boolean _useDuration = false;
    private double _duration = 0;
    private boolean _lock = false;

    public LedSubsystem() {
        this._led = new AddressableLED(Constants.LED.Port);
        // The actual color order is GRBW, but we have it set to RGB so it's easier to visualize the
        // color order in the code below.
        this._led.setColorOrder(ColorOrder.kRGB);
        this._ledBuffer = new AddressableLEDBuffer(Constants.LED.StripLength);
        this._led.setLength(this._ledBuffer.getLength());
        this._led.setData(this._ledBuffer);
        this._led.start();
        this._timer.reset();
        this._timer.start();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (_useDuration && _timer.get() >= _duration) {
            _useDuration = false;
            _duration = 0.0;
            reset();
        }

        if (!_lock) reset();
    }

    private void SetColor(Color color) {
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
    }

    public LedSubsystem setColor(Color color) {
        _lock = false;
        SetColor(color);
        return this;
    }

    public LedSubsystem setColor(Color color, boolean lock) {
        _lock = true;
        SetColor(color);
        return this;
    }

    public LedSubsystem reset() {
        _lock = false;
        SetColor(kDefaultInactiveColor);
        _timer.restart();
        return this;
    }

    public LedSubsystem off() {
        LEDPattern.solid(kDefaultInactiveColor).applyTo(_ledBuffer);
        _led.setData(_ledBuffer);
        return this;
    }

    /**
     * Runs the LED strip for a set color and duration.
     * @param color
     * @param duration Seconds
     * @return
     */
    public LedSubsystem setColorForDuration(Color color, double duration) {
        _timer.restart();
        setColor(color, true);
        _useDuration = true;
        _duration = Math.max(duration, 0);
        return this;
    }
}