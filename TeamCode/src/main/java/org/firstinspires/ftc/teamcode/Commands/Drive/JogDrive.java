package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.Drive_Subsystem;


public class JogDrive extends CommandBase {
    private Drive_Subsystem drive;

    private GamepadEx gamepad;


    public JogDrive(Drive_Subsystem drive, GamepadEx gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (drive.fieldCentric) {
            drive.drive.driveFieldCentric(
                    this.gamepad.getLeftX(),
                    this.gamepad.getLeftY(),
                    this.gamepad.getRightX(),
                    drive.imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                    false
            );

        } else {
            drive.drive.driveRobotCentric(
                    this.gamepad.getLeftX(),
                    this.gamepad.getLeftY(),
                    this.gamepad.getRightX(), false
            );
        }



    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
