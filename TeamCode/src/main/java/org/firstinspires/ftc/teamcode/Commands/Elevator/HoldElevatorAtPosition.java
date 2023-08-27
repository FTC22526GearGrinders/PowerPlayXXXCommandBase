package org.firstinspires.ftc.teamcode.Commands.Elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;


public class HoldElevatorAtPosition extends CommandBase {
    private Elevator_Subsystem elevator;

    private double power;

    private double distance;

    private int counts;


    public HoldElevatorAtPosition(Elevator_Subsystem elevator) {
        this.elevator = elevator;

        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        distance=elevator.getPosition();
    }

    @Override
    public void execute() {
        double output = elevator.controller.calculate(
                elevator.getPosition(), this.distance);  // the measured value

        double motorOutput = output;



        elevator.setPower(motorOutput + Constants.ElevatorConstants.POSITION_Kg);


    }


    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
