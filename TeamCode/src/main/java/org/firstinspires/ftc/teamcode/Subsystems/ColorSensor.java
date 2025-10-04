package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Wrapper class for FTC ColorSensor.
 *
 * Provides:
 * - Raw RGBA values
 * - Normalized color (0..1)
 * - HSV conversion
 * - Dominant color classification (RED, BLUE, GREEN, NONE)
 * - Simple "is object present" detection
 */
public class ColorSensorSubsystem {

    private final ColorSensor colorSensor;

    // Scale factor for normalization (REV sensors often return 0–800+)
    private static final double SCALE_FACTOR = 255.0;

    // Threshold for object detection (tune per robot/environment)
    private static final int DETECTION_THRESHOLD = 200;

    public enum DetectedColor {
        RED, BLUE, GREEN, UNKNOWN
    }

    public ColorSensorSubsystem(HardwareMap hardwareMap, String sensorName) {
        this.colorSensor = hardwareMap.get(ColorSensor.class, sensorName);
    }

    /** Get raw red value. */
    public int getRawRed() { return colorSensor.red(); }

    /** Get raw green value. */
    public int getRawGreen() { return colorSensor.green(); }

    /** Get raw blue value. */
    public int getRawBlue() { return colorSensor.blue(); }

    /** Get raw alpha (brightness) value. */
    public int getRawAlpha() { return colorSensor.alpha(); }

    /** Return normalized [0..1] RGB values. */
    public float[] getNormalizedRGB() {
        float[] rgb = new float[3];
        rgb[0] = (float) (getRawRed()   / SCALE_FACTOR);
        rgb[1] = (float) (getRawGreen() / SCALE_FACTOR);
        rgb[2] = (float) (getRawBlue()  / SCALE_FACTOR);
        return rgb;
    }

    /** Convert to HSV (Hue-Saturation-Value). */
    public float[] getHSV() {
        float[] hsv = new float[3];
        Color.RGBToHSV(getRawRed(), getRawGreen(), getRawBlue(), hsv);
        return hsv; // hsv[0]=hue(0-360), hsv[1]=sat(0-1), hsv[2]=val(0-1)
    }

    /** Simple heuristic for object presence: alpha or sum of RGB above threshold. */
    public boolean isObjectDetected() {
        return (getRawAlpha() > DETECTION_THRESHOLD);
    }

    /** Determine dominant color (RED, BLUE, GREEN). */
    public DetectedColor getDominantColor() {
        int r = getRawRed();
        int g = getRawGreen();
        int b = getRawBlue();

        if (r > g && r > b && r > DETECTION_THRESHOLD) {
            return DetectedColor.RED;
        } else if (b > r && b > g && b > DETECTION_THRESHOLD) {
            return DetectedColor.BLUE;
        } else if (g > r && g > b && g > DETECTION_THRESHOLD) {
            return DetectedColor.GREEN;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }
}
