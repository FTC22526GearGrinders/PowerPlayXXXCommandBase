package org.firstinspires.ftc.teamcode.Commands.Elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;


public class JogElevatorPlus extends CommandBase {
    private Elevator_Subsystem elevator;

    public JogElevatorPlus(Elevator_Subsystem elevator) {
        this.elevator = elevator;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (elevator.getPosition() < Constants.ElevatorConstants.UPPER_POSITION_LIMIT) {
            elevator.setPower(Constants.ElevatorConstants.JOG_UP_POWER);
            elevator.targetPosition = elevator.getPosition();

        }

        else elevator.setPower(Constants.ElevatorConstants.POSITION_Kg);
    }
        @Override
        public void end ( boolean interrupted){

        }

        @Override
        public boolean isFinished () {
            return false;
        }
    }
