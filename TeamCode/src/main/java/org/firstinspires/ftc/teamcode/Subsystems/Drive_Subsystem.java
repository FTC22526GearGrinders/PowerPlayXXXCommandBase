package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;


import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.MecanumDriveKinematics;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.MecanumDriveOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.MecanumDriveWheelPositions;
import org.firstinspires.ftc.teamcode.Utils.SlewRateLimiter;

import java.lang.reflect.Field;


public class Drive_Subsystem extends SubsystemBase {
    public boolean fieldCentric;
    public double roterror;
    public double distError;






    double wheelWidth   = Constants.DriveConstants.WHEELWIDTH;
    double trackWidth= Constants.DriveConstants.TRACKWIDTH;
    // Locations of the wheels relative to the robot center.
    Translation2d m_frontLeftLocation =
            new Translation2d(wheelWidth / 2, trackWidth / 2);
    Translation2d m_frontRightLocation =
            new Translation2d(wheelWidth / 2, -trackWidth / 2);
    Translation2d m_backLeftLocation =
            new Translation2d(-wheelWidth / 2, trackWidth / 2);
    Translation2d m_backRightLocation =
            new Translation2d(-wheelWidth / 2, -trackWidth / 2);

    Pose2d m_pose;


    public Motor frontleftmotor;
    public Motor frontrightmotor;
    public Motor backleftmotor;
    public Motor backrightmotor;
    public Motor.Encoder m_frontLeftEncoder;
    public Motor.Encoder m_frontRightEncoder;
    Motor.Encoder m_backLeftEncoder;
    Motor.Encoder m_backRightEncoder;

    private int test;

    private ElapsedTime runtime = new ElapsedTime();


    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics
            (
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation
            );

    MecanumDriveOdometry m_odometry;

    public MecanumDriveWheelPositions wheelPositions;


    public MecanumDrive drive;

    DifferentialDrive arcadeDrive;

    MotorGroup leftGroup;

    MotorGroup rightGroup;


    private SlewRateLimiter slr = new SlewRateLimiter(.25, -.25, 0);

    public double targetDistance;

    public double targetAngle;

    private double rotatekP = Constants.DriveConstants.ROTATE_Kp;

    public PIDController controller = new PIDController(Constants.DriveConstants.POSITION_Kp, Constants.DriveConstants.POSITION_Ki, Constants.DriveConstants.POSITION_Kd);


    private double positionPower;

    public RevIMU imu;


    CommandOpMode myOpmode;


    // This is the built-in IMU in the REV hub.
    // We're initializing it by its default parameters
    // and name in the config ('imu'). The orientation
    // of the hub is important. Below is a model
    // of the REV Hub and the orientation axes for the IMU.
    //
    //                           | Z axis
    //                           |
    //     (Motor Port Side)     |   / X axis
    //                       ____|__/____
    //          Y axis     / *   | /    /|   (IO Side)
    //          _________ /______|/    //      I2C
    //                   /___________ //     Digital
    //                  |____________|/      Analog
    //
    //                 (Servo Port Side)
    //
    // (unapologetically stolen from the road-runner-quickstart)

    public Drive_Subsystem(CommandOpMode opMode) {
        myOpmode = opMode;
        runtime.reset();
        frontleftmotor = new Motor(myOpmode.hardwareMap, "left motor", Motor.GoBILDA.RPM_312);
        frontrightmotor = new Motor(myOpmode.hardwareMap, "right motor", Motor.GoBILDA.RPM_312);
        backleftmotor = new Motor(myOpmode.hardwareMap, "left back", Motor.GoBILDA.RPM_312);
        backrightmotor = new Motor(myOpmode.hardwareMap, "right back", Motor.GoBILDA.RPM_312);
        frontleftmotor.setInverted(true);
        frontrightmotor.setInverted(true);
        backleftmotor.setInverted(true);
        backrightmotor.setInverted(true);

        frontleftmotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setDistancePerPulse(Constants.DriveConstants.ENCODER_COUNTS_PER_INCH);
        frontleftmotor.setDistancePerPulse(Constants.DriveConstants.ENCODER_COUNTS_PER_INCH);
        backrightmotor.setDistancePerPulse(Constants.DriveConstants.ENCODER_COUNTS_PER_INCH);
        backleftmotor.setDistancePerPulse(Constants.DriveConstants.ENCODER_COUNTS_PER_INCH);


        m_frontLeftEncoder = frontleftmotor.encoder;
        m_frontRightEncoder = frontrightmotor.encoder;
        m_backLeftEncoder = backleftmotor.encoder;
        m_backRightEncoder = backrightmotor.encoder;

        m_frontRightEncoder.setDirection(Motor.Direction.REVERSE);
        m_backRightEncoder.setDirection(Motor.Direction.REVERSE);
        m_frontLeftEncoder.setDistancePerPulse(Constants.DriveConstants.ENCODER_COUNTS_PER_INCH);
        m_frontRightEncoder.setDistancePerPulse(Constants.DriveConstants.ENCODER_COUNTS_PER_INCH);
        m_backLeftEncoder.setDistancePerPulse(Constants.DriveConstants.ENCODER_COUNTS_PER_INCH);
        m_backRightEncoder.setDistancePerPulse(Constants.DriveConstants.ENCODER_COUNTS_PER_INCH);


        imu = new RevIMU(myOpmode.hardwareMap);

        imu.init();
        reset();
        drive = new MecanumDrive(frontleftmotor, frontrightmotor, backleftmotor, backrightmotor);

        wheelPositions = new MecanumDriveWheelPositions(m_frontLeftEncoder.getDistance(),m_frontRightEncoder.getDistance(),m_backLeftEncoder.getDistance(),m_backRightEncoder.getDistance());


        m_odometry = new MecanumDriveOdometry
                (
                        m_kinematics, getGyroHeading(), wheelPositions,

                        new Pose2d()
                );
        runtime.reset();

//        leftGroup = new MotorGroup(frontleftmotor, backleftmotor);
//
//        rightGroup = new MotorGroup(frontrightmotor, backrightmotor);

    }

    public void periodic() {
        test++;


        // Get my wheel speeds; assume .getRate() has been
        // set up to return velocity of the encoder
        // in meters per second.


        // Update the pose);
        m_pose = m_odometry.update(getGyroHeading(), wheelPositions);
        //myOpmode.telemetry.addLine("Front Encoders Inches|")
        //     .addData("FL", "%.2f", wheelSpeeds.frontLeftMetersPerSecond);
        //   .addData("|FR ", "%.2f", wheelSpeeds.frontRightMetersPerSecond);
//        myOpmode.telemetry.addLine("Rear Encoders |")
//                .addData("BL", "%.2f", wheelSpeeds.rearLeftMetersPerSecond)
//                .addData("BR ", "%.2f", wheelSpeeds.rearRightMetersPerSecond);
        //myOpmode.telemetry.update();
    }


    public double getAverageFrontDistance() {
        return (m_frontLeftEncoder.getDistance() + m_frontRightEncoder.getDistance()) / 2;
    }

    public double getAverageBackDistance() {
        return (m_backRightEncoder.getDistance() + m_backLeftEncoder.getDistance()) / 2;
    }

    public void setPositionKp(double kp) {
        controller.setP(kp);
    }

    public double getPositionKp() {
        return controller.getP();
    }

    public void reset() {

        m_frontLeftEncoder.reset();
        m_frontRightEncoder.reset();
        m_backLeftEncoder.reset();
        m_backRightEncoder.reset();
        imu.reset();
    }

    public double getPower() {
        return frontleftmotor.get();
    }

    public void setP(double kp) {
        if (getP() != kp)
            controller.setP(kp);
    }

    public double getP() {
        return controller.getP();
    }

    public void setRotatekP(double kp) {
        if (rotatekP != kp)
            rotatekP = kp;
    }

    public double getRotatekP() {
        return rotatekP;
    }

    public Rotation2d getGyroHeading() {
        return imu.getRotation2d();
    }

    public void toggleFieldOrient() {
        if (fieldCentric) fieldCentric = false;

        else fieldCentric = true;
    }


    public void showTelemetry() {
//
//
        myOpmode.telemetry.addData("frontleft", m_frontLeftEncoder.getDistance());
        myOpmode.telemetry.addData("frontleft", m_frontRightEncoder.getDistance());
        myOpmode.telemetry.addData("frontleft", m_backRightEncoder.getDistance());
        myOpmode.telemetry.addData("frontleft", m_backLeftEncoder.getDistance());
//        myOpmode.telemetry.addData("api", Constants.DriveConstants.ENCODER_COUNTS_PER_INCH);
//        myOpmode.telemetry.addData("Gyro Angle", "%.2f", getGyroHeading().getDegrees());
////        myOpmode.telemetry.addData("target", targetDistance);
////        myOpmode.telemetry.addData("power", positionPower);
////        myOpmode.telemetry.addData("TargetAngle", targetAngle);
////        //myOpmode.telemetry.addData("roterror",roterror);
////        // myOpmode.telemetry.addData("disterror",distError);
////        //myOpmode.telemetry.addData("fieldcentric",fieldCentric);
        myOpmode.telemetry.addData("X", m_pose.getX());
        myOpmode.telemetry.addData("Y", m_pose.getY());
//        myOpmode.telemetry.addData("RTS", runtime.seconds());
//
//
         myOpmode.telemetry.update();
    }
}



