package org.firstinspires.ftc.teamcode.subsystems.drive.tank;

import static org.apache.commons.math3.stat.StatUtils.normalize;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.subsystems.drive.tank.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.tank.rrExt.ImprovedRamsete;
import org.firstinspires.ftc.teamcode.subsystems.drive.tank.rrExt.ImprovedTankDrive;
import org.firstinspires.ftc.teamcode.util.misc.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.misc.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple tank drive hardware implementation for REV hardware.
 */
@Config
//TODO:FIX!!!!!
public class Drivetrain extends ImprovedTankDrive implements Subsystem {
    //public static PIDCoefficients AXIAL_PID = new PIDCoefficients(5, 0, 0); //7,0,1
    //public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0.04, 0, 0); //0.06,0,0
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(14, 0, 0.6); //14.5,0,1
    /*
        public static PIDCoefficients LEFT_DRIVE_PID = new PIDCoefficients(0.01, 0, 0);
        public static PIDCoefficients RIGHT_DRIVE_PID = new PIDCoefficients(0.01, 0, 0);
     */
    private PIDController leftDriveVeloPID;
    private PIDController rightDriveVeloPID;

    public static double VX_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private List<DcMotorEx> motors, leftMotors, rightMotors;
    private IMU imu;

    private VoltageSensor batteryVoltageSensor;

    public Drivetrain(HardwareMap hardwareMap, boolean isEnabled) {
        super(kV, kA, kStatic, TRACK_WIDTH, hardwareMap.voltageSensor.iterator().next());

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
/*
        leftDriveVeloPID = new PIDController(LEFT_DRIVE_PID.kP, LEFT_DRIVE_PID.kI, LEFT_DRIVE_PID.kD);
        rightDriveVeloPID = new PIDController(RIGHT_DRIVE_PID.kP, RIGHT_DRIVE_PID.kI, RIGHT_DRIVE_PID.kD);
 */
        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new TankVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        /*
            follower = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
         */

        follower = new ImprovedRamsete();

        poseHistory = new ArrayList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // add/remove motors depending on your robot (e.g., 6WD)
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        leftMotors = Arrays.asList(leftFront, leftRear);
        rightMotors = Arrays.asList(rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            //new
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }
    
    public void arcadeDrive(double forward, double rotate) {
        double maxInput = Math.copySign(Math.max(Math.abs(forward), Math.abs(rotate)), forward);
//        forward = clipRange(forward);
//        rotate = clipRange(rotate);
        
        double[] wheelSpeeds = new double[2];
        wheelSpeeds[0] = forward + rotate;
        wheelSpeeds[1] = forward - rotate;
        
        normalize(wheelSpeeds);
        
        setMotorPowers(wheelSpeeds[0], wheelSpeeds[1]);
    }
    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnToAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        poseHistory.clear();
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    0,
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }
    /*
        @Override
        public void setDriveSignal(@NotNull DriveSignal driveSignal) {
            List<Double> velocities = TankKinematics.robotToWheelVelocities(driveSignal.getVel(), TRACK_WIDTH);
            List<Double> accelerations = TankKinematics.robotToWheelVelocities(driveSignal.getAccel(), TRACK_WIDTH);
            List<Double> feedforwards = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
            leftDriveVeloPID.setPID(LEFT_DRIVE_PID.kP, LEFT_DRIVE_PID.kI, LEFT_DRIVE_PID.kD);
            rightDriveVeloPID.setPID(RIGHT_DRIVE_PID.kP, RIGHT_DRIVE_PID.kI, RIGHT_DRIVE_PID.kD);
            DriveVeloPID.setPID(RIGHT_DRIVE_PID.kP, RIGHT_DRIVE_PID.kI, RIGHT_DRIVE_PID.kD);
            double leftOutput = feedforwards.get(0) + leftDriveVeloPID.calculate(getWheelVelocities().get(0), velocities.get(0));
            double rightOutput = feedforwards.get(1) + leftDriveVeloPID.calculate(getWheelVelocities().get(1), velocities.get(1));
            setMotorPowers(leftOutput, rightOutput);
        }
     */
    @NonNull
    @Override
    public List<Double> getWheelPositions() {

            double leftSum = 0, rightSum = 0;
            for (DcMotorEx leftMotor : leftMotors) {
                leftSum += encoderTicksToInches(leftMotor.getCurrentPosition());
            }
            for (DcMotorEx rightMotor : rightMotors) {
                rightSum += encoderTicksToInches(rightMotor.getCurrentPosition());
            }
            return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());

        /*double leftSum = encoderTicksToInches(leftMotors.get(0).getCurrentPosition());
        double rightSum = -encoderTicksToInches(rightMotors.get(0).getCurrentPosition());

        return Arrays.asList(leftSum, rightSum);*/
    }

    public List<Double> getWheelVelocities() {
        double leftSum = encoderTicksToInches(leftMotors.get(0).getVelocity());
        double rightSum = -encoderTicksToInches(rightMotors.get(0).getVelocity());

        return Arrays.asList(leftSum, rightSum);
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }
    
    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    
    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }
    
    public void stop(){
        arcadeDrive(0,0);
    }
}