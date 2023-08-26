package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive_Subsystem;
import org.firstinspires.ftc.teamcode.Utils.SlewRateLimiter;


public class PositionDrive extends CommandBase {
    private Drive_Subsystem drive;
    private ElapsedTime stopTime = new ElapsedTime();

    private double distance;

    private GamepadEx gamepad;

    private SlewRateLimiter slr ;
    private double startingAngle;
    private double distError;

    public PositionDrive(Drive_Subsystem drive, double distance) {
        this.drive = drive;
        this.distance = distance;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        startingAngle = drive.getGyroHeading().getDegrees();
        drive.controller.setTolerance(Constants.DriveConstants.POSITION_TOLERANCE);
        drive.targetDistance = distance;

        if (this.distance< drive.getAverageFrontDistance())
            slr = new SlewRateLimiter(999999,-.25,0);
        else slr = new SlewRateLimiter(.25,-999999,0);
    }

    @Override
    public void execute() {

        drive.distError = drive.controller.calculate(drive.getAverageFrontDistance(), this.distance);

        double output = drive.distError;

        if (Math.abs(output) > Constants.DriveConstants.POSITION_POWER) {
            output = Math.signum(output) * Constants.DriveConstants.POSITION_POWER;
        }

        double temp1 = slr.calculate(output);

        drive.drive.driveRobotCentric(0, temp1, ((startingAngle + drive.getGyroHeading().getDegrees()) * Constants.DriveConstants.MOVE_STRAIGHT_Kp), false);

  if(!drive.controller.atSetPoint())stopTime.reset();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return stopTime.seconds()>=1;
    }
}
