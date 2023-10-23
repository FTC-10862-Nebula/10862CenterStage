package org.firstinspires.ftc.teamcode.subsystems.sensor;

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
        telemetry.addData("\tGreen:", colorSensor.green());
        telemetry.addData("\tBlue:", colorSensor.blue());

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

    public boolean grabbedBlueCone() {
//        telemetry.addLine("Got Blue Cone");
        return (colorSensor.blue() > 500);
    }
    public boolean grabbedRedCone() {
//        telemetry.addLine("Got Red Cone");
        return (colorSensor.red() > 500);
    }
}