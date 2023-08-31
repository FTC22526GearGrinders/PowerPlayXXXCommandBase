package org.firstinspires.ftc.teamcode.Commands.Utils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TimeDelay extends CommandBase {

    private ElapsedTime delayTimer = new ElapsedTime();

    private final double delaySecs;

    public TimeDelay(double delaySec) {
        this.delaySecs = delaySec;
    }

    @Override
    public void initialize() {
        delayTimer.reset();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return delayTimer.seconds() > delaySecs;
    }
}
