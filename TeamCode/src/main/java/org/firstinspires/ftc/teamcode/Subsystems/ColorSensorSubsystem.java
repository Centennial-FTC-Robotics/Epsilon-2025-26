package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * ColorSensor subsystem for FTC Decode Challenge.
 * Detects GREEN or PURPLE artifacts using HSV hue ranges.
 */
public class ColorSensorSubsystem {

    private final ColorSensor colorSensor;

    // TUNE THESE
    private static final float GREEN_HUE_MIN = 90f;
    private static final float GREEN_HUE_MAX = 160f;

    private static final float PURPLE_HUE_MIN = 250f;
    private static final float PURPLE_HUE_MAX = 300f;

    // Alpha checks for if ball exists
    private static final int DETECTION_THRESHOLD = 150;

    public enum DetectedColor {
        GREEN, PURPLE, UNKNOWN
    }

    public ColorSensorSubsystem(HardwareMap hardwareMap, String sensorName) {
        this.colorSensor = hardwareMap.get(ColorSensor.class, sensorName);
    }

    public int getRed() {
        return colorSensor.red();
    }
    public int getGreen() {
        return colorSensor.green();
    }
    public int getBlue() {
        return colorSensor.blue();
    }
    public int getAlpha() {
        return colorSensor.alpha();
    }

    private float[] getHSV() {
        float[] hsv = new float[3];
        Color.RGBToHSV(getRed(), getGreen(), getBlue(), hsv);
        return hsv;
    }

    public boolean isObjectDetected() {
        return getAlpha() > DETECTION_THRESHOLD;
    }

    /** Detect whether the artifact is GREEN or PURPLE */
    public DetectedColor getDetectedColor() {
        if (!isObjectDetected()) return DetectedColor.UNKNOWN;

        float[] hsv = getHSV();
        float hue = hsv[0];

        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            return DetectedColor.GREEN;
        } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }
}