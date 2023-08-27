package org.firstinspires.ftc.teamcode.Commands.Elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utils.SlewRateLimiter;


public class PositionElevator extends CommandBase {
    private Elevator_Subsystem elevator;


    private double distance;

    private int counts;

    private SlewRateLimiter slr = new SlewRateLimiter(.1, -.1, 0);
private ElapsedTime postime = new ElapsedTime();

    public PositionElevator(Elevator_Subsystem elevator, double distance) {
        this.elevator = elevator;
        this.distance = distance;

        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        slr.reset(0);
        elevator.targetPosition = distance;

    }

    @Override
    public void execute() {

        double output = elevator.controller.calculate(
                elevator.getPosition(), elevator.targetPosition);  // the measured value

        if (output > 0 && output > Constants.ElevatorConstants.POSITION_UP_POWER)
            output = Constants.ElevatorConstants.POSITION_UP_POWER;

        if (output < 0 && output < Constants.ElevatorConstants.POSITION_DOWN_POWER)
            output = Constants.ElevatorConstants.POSITION_DOWN_POWER;

        double motorOutput =output;// slr.calculate(output);

        elevator.setPower(motorOutput);


    if (!elevator.inPosition())postime.reset();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return postime.seconds() > .5;
    }
}
