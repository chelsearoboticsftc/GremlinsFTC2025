package org.firstinspires.ftc.teamcode.subsystems; // Make sure this matches your team's package name

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensorProcessor {

    // Enum to represent the detected colors
    public enum DetectedColor {
        GREEN,
        PURPLE,
        NONE
    }

    public ColorSensor colorSensor;

    // target hues found in org.firstinspires.ftc.vision.opencv.PredominantColorProcessor
    private static final float GREEN_HUE = 160.0f;

    private static final float PURPLE_HUE = 210.0f;

    private static final float HUE_TOLERANCE = 30.0f;

    // Minimum saturation and value to be considered a color
    private static final float MIN_SATURATION = 0.2f;
    private static final float MIN_VALUE = 1f;


    // hsvValues is an array that will hold the hue, saturation, and value information.
    public float[] hsvValues = {0F, 0F, 0F};

    public ColorSensorProcessor(HardwareMap hardwareMap, String sensorName) {
        colorSensor = hardwareMap.get(ColorSensor.class, sensorName);
        // make sure the LED is on
        colorSensor.enableLed(true);
    }

    /**
     * Detects if the color is Orange, Purple, or None.
     * @return A DetectedColor enum value.
     */
    public DetectedColor getDetectedColor() {
        // Get the raw RGB values, and normalize them to be between 0 and 1
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Convert the RGB colors to HSV color model.
        // See: https://en.wikipedia.org/wiki/HSL_and_HSV
        Color.RGBToHSV(red, green, blue, hsvValues);

        // hsvValues[0] is Hue (0-360)
        // hsvValues[1] is Saturation (0-1)
        // hsvValues[2] is Value (0-1)
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        // Check if the color is "strong" enough to be considered
        if (saturation < MIN_SATURATION || value < MIN_VALUE) {
            return DetectedColor.NONE;
        }

        // Check if the hue is within our defined orange range
        // Check if the hue is within our defined purple range
        if (hue >= PURPLE_HUE - HUE_TOLERANCE && hue <= PURPLE_HUE + HUE_TOLERANCE) {
            return DetectedColor.PURPLE;
        }
        else if (hue >= GREEN_HUE - HUE_TOLERANCE && hue <= GREEN_HUE + HUE_TOLERANCE) {
            return DetectedColor.GREEN;
        }

        // If it's neither, return NONE
        return DetectedColor.NONE;
    }
}