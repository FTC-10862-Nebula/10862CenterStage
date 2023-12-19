package org.firstinspires.ftc.teamcode.subsystems.sensor;

import android.graphics.ColorSpace;
import android.provider.CalendarContract;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SensorColor extends SubsystemBase {

    private final ColorSensor colorSensor;
    private final Telemetry telemetry;

    public SensorColor(HardwareMap hardwareMap , Telemetry tl) {
        this.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        this.telemetry = tl;

        this.colorSensor.enableLed(true);
    }

    public void periodic() {
//        telemetry.addData("Distance (cm)",
//                String.format(Locale.US, "%.02f", colorSensor.getDistance(DistanceUnit.INCH)));
        telemetry.addData("\tAlpha:", colorSensor.alpha());
        telemetry.addData("\tRed:", colorSensor.red());
        telemetry.addData("\tBlue:", colorSensor.blue());
        telemetry.addData("\tGreen:", colorSensor.green());


        telemetry.update();
    }

//    public int[] HSVtoARGB(int alpha, float[] hsv) {
//        int color = Color.HSVToColor(alpha, hsv);
//        return new int[]{Color.alpha(color), Color.red(color), Color.green(color), Color.blue(color)};
//    }
//
//    public float[] RGBtoHSV(int red, int green, int blue, float[] hsv) {
//        Color.RGBToHSV(red, green, blue, hsv);
//        return hsv;
//    }

    public boolean grabbedWhitePixel() {
//        telemetry.addLine("Got White pixel");
        return (colorSensor.blue() > 255 &&colorSensor.red()> 255 &&colorSensor.green()> 255);
        //change to white
    }
    public boolean grabbedRedCone() {
//        telemetry.addLine("Got Purple Pixel");
        return (colorSensor.red() > 165 &&colorSensor.blue() > 142 &&colorSensor.green()>218);
        //change to  purple
    }
    public boolean grabbedGreenPixel() {
//        telemetry.addLine("Got Green pixel");
        return (colorSensor.blue() > 36 &&colorSensor.red() > 39 &&colorSensor.green()> 171);
        // change to green
    }
    public boolean grabbedYellowPixel() {
//        telemetry.addLine("Got Yellow pixel");
        return (colorSensor.blue() > 11 &&colorSensor.red() > 246 &&colorSensor.green()> 183);
        //change to yellow
    }
}