package org.firstinspires.ftc.teamcode.subsystems.drive;

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
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
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
import org.firstinspires.ftc.teamcode.util.misc.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.misc.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/*
 * Simple tank drive hardware implementation for REV hardware.
 */
@Config
public class SixWheel extends ImprovedTankDrive {
    //public static PIDCoefficients AXIAL_PID = new PIDCoefficients(5, 0, 0); //7,0,1
    //public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0.04, 0, 0); //0.06,0,0
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(14, 0, 0.6); //14.5,0,1
    /*
        public static PIDCoefficients LEFT_DRIVE_PID = new PIDCoefficients(0.01, 0, 0);
        public static PIDCoefficients RIGHT_DRIVE_PID = new PIDCoefficients(0.01, 0, 0);
     */
    private PIDController leftDriveVeloPID;
    private PIDController rightDriveVeloPID;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL);


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

    //TODO:
//    private TrajectorySequenceRunner trajectorySequenceRunner = new TrajectorySequenceRunner();
    private TrajectorySequenceRunner trajectorySequenceRunner;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private List<DcMotorEx> motors, leftMotors, rightMotors;
    private IMU imu;

    private VoltageSensor batteryVoltageSensor;

    public SixWheel(HardwareMap hardwareMap) {
        super(DriveConstants.kV,
            DriveConstants.kA,
            DriveConstants.kStatic,
            DriveConstants.TRACK_WIDTH,
            hardwareMap.voltageSensor.iterator().next());

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
                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                new TankVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);
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

        if (DriveConstants.RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (DriveConstants.RUN_USING_ENCODER && DriveConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
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
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                DriveConstants.MAX_ANG_VEL,
                DriveConstants.MAX_ANG_ACCEL
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnToAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(angle, 0, 0, 0),
                DriveConstants.MAX_ANG_VEL,
                DriveConstants.MAX_ANG_ACCEL
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
                leftSum += DriveConstants.encoderTicksToInches(leftMotor.getCurrentPosition());
            }
            for (DcMotorEx rightMotor : rightMotors) {
                rightSum += DriveConstants.encoderTicksToInches(rightMotor.getCurrentPosition());
            }
            return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());

        /*double leftSum = encoderTicksToInches(leftMotors.get(0).getCurrentPosition());
        double rightSum = -encoderTicksToInches(rightMotors.get(0).getCurrentPosition());

        return Arrays.asList(leftSum, rightSum);*/
    }

    public List<Double> getWheelVelocities() {
        double leftSum = DriveConstants.encoderTicksToInches(leftMotors.get(0).getVelocity());
        double rightSum = -DriveConstants.encoderTicksToInches(rightMotors.get(0).getVelocity());

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
    public double getDegreePitch() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }
    public double getDegreeRoll() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }
    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public void resetImu() {
        imu.resetYaw();
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }


    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }
}