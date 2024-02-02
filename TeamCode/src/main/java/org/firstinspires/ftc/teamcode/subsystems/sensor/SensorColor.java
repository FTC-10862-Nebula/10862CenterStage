package org.firstinspires.ftc.teamcode.subsystems.sensor;

import android.graphics.ColorSpace;
import android.provider.CalendarContract;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SensorColor extends SubsystemBase {

    private final ColorSensor colorSensorF, colorSensorB;
    private final Telemetry telemetry;

    public SensorColor(Telemetry tl, HardwareMap hardwareMap) {

        this.colorSensorF = hardwareMap.get(ColorSensor.class, "colorSF");
        this.colorSensorB = hardwareMap.get(ColorSensor.class, "colorSB");
        this.telemetry = tl;
    }

    public void periodic() {
//        telemetry.addData("Distance (cm)",
//                String.format(Locale.US, "%.02f", colorSensor.getDistance(DistanceUnit.INCH)));
//        telemetry.addData("\tFAlpha:", colorSensorF.alpha());
//        telemetry.addData("\tFRed:", colorSensorF.red());
//        telemetry.addData("\tFBlue:", colorSensorF.blue());
//        telemetry.addData("\tFGreen:", colorSensorF.green());
//        telemetry.addLine();
//        telemetry.addData("\tBAlpha:", colorSensorB.alpha());
//        telemetry.addData("\tBRed:", colorSensorB.red());
//        telemetry.addData("\tBBlue:", colorSensorB.blue());
//        telemetry.addData("\tBGreen:", colorSensorB.green());

        telemetry.addData("grabBackPixel: ", grabbedBackPixel());
        telemetry.addData("grabFrontPixel: ", grabbedFrontPixel());


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

    public boolean grabbedWhitePixel(ColorSensor sensor) {
//        telemetry.addLine("Got White pixel");
        return (sensor.blue() > 2000 &&sensor.red()> 2000 &&sensor.green()> 2000);
        //change to white
    }
    public boolean grabbedPurpleCone(ColorSensor sensor) {
//        telemetry.addLine("Got Purple Pixel");
        return (sensor.red() > 1000 &&sensor.blue() > 1500 &&sensor.green()>1000);
        //change to purple
    }
    public boolean grabbedGreenPixel(ColorSensor sensor) {
//        telemetry.addLine("Got Green pixel");
        return ( sensor.green()> 1000);
        // change to green
    }
    public boolean grabbedYellowPixel(ColorSensor sensor) {
//        telemetry.addLine("Got Yellow pixel");
        return (sensor.blue() > 250 &&sensor.red() > 1000 &&sensor.green()> 1000);
        //change to yellow
    }
    public boolean grabbedBackPixel() {
        return grabbedYellowPixel(colorSensorB)||grabbedGreenPixel(colorSensorB)||
                grabbedPurpleCone(colorSensorB)||grabbedWhitePixel(colorSensorB);
    }
    public boolean grabbedFrontPixel() {
        return grabbedYellowPixel(colorSensorF)||grabbedGreenPixel(colorSensorF)||
                grabbedPurpleCone(colorSensorF)||grabbedWhitePixel(colorSensorF);
    }

}