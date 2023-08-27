package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Constants;

public class Elevator_Subsystem extends SubsystemBase {

    public Motor elevatorMotor;

    public Motor.Encoder elevatorEncoder;



    private CommandOpMode myOpMode;

    double kP = Constants.ElevatorConstants.POSITION_Kp;
    double kI = 0;
    double kD = Constants.ElevatorConstants.POSITION_Kd;

    public PIDController controller = new PIDController(kP, kI, kD);

    public double targetPosition;


    public Elevator_Subsystem(CommandOpMode opMode) {
        myOpMode = opMode;

        elevatorMotor = new Motor(myOpMode.hardwareMap, "elivarorshaft 1", Motor.GoBILDA.RPM_435);
        elevatorMotor.setInverted(true);
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        elevatorEncoder = elevatorMotor.encoder;

        //elevatorEncoder.setDirection(Motor.Direction.REVERSE);

        elevatorEncoder.setDistancePerPulse(1/Constants.ElevatorConstants.ENCODER_COUNTS_PER_INCH);

        controller.setTolerance(Constants.ElevatorConstants.POSITION_TOLERANCE);

        resetEncoder();

    }



    public void resetEncoder() {
        elevatorEncoder.reset();

    }

    @Override
    public void periodic() {

    }


    public void holdPositionVolts() {

    }

    public void holdPositionLock() {

        double output = controller.calculate(
                getPosition(), targetPosition);  // the measured value
        elevatorMotor.set(output + Constants.ElevatorConstants.POSITION_Kg);
    }

    public void setPositionKp(double kp) {
        controller.setP(kp);
    }

    public double getPositionKp() {
        return controller.getP();
    }

    public boolean inPosition() {
        return controller.atSetPoint();
    }

    public double getRate() {
        return elevatorEncoder.getRate();

    }
public void setPower(double power){
        elevatorMotor.set(power);
    }

    public double getPower() {
        return elevatorMotor.get();
    }

    public double getPosition() {
        return elevatorEncoder.getDistance();
    }



    public void showTelemetry() {

        myOpMode.telemetry.addData("El Posn", getPosition());
        myOpMode.telemetry.addData("El InPod", inPosition());
        myOpMode.telemetry.addData("El Pow", getPower());
        myOpMode.telemetry.addData("El rate", getRate());
       myOpMode. telemetry.addData("Target", targetPosition);
        myOpMode.telemetry.update();
    }

}