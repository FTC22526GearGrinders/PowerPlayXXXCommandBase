package org.firstinspires.ftc.teamcode.Commands.Elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;


public class JogElevatorMinus extends CommandBase {
    private Elevator_Subsystem elevator;

    public JogElevatorMinus(Elevator_Subsystem elevator) {
        this.elevator = elevator;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        if (elevator.getPosition() > Constants.ElevatorConstants.LOWER_POSITION_LIMIT) {
            elevator.setPower(Constants.ElevatorConstants.JOG_DOWN_POWER);
            elevator.targetPosition = elevator.getPosition();
        }
        else elevator.setPower(Constants.ElevatorConstants.POSITION_Kg);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
