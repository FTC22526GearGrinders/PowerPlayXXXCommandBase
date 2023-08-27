package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive_Subsystem;

//Rev IMU give angles +!80 to -180
//Angle will be commanded the same way
// shortest route motion calculated by error = command - actual

// if (angleError >= 180) angleError -= 360;

// if (angleError <= 180) angleError += 360;

//above gives direction and size of error


public class RotateToAngle extends CommandBase {
    private Drive_Subsystem drive;
    private ElapsedTime rottimer = new ElapsedTime();

    public double kP;

    double error = 0;
    private double angle;

    double angleError;

    private ElapsedTime timeout = new ElapsedTime();

    public RotateToAngle(Drive_Subsystem drive, double angle) {
        this.drive = drive;
        this.angle = angle;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        //Make angle between 0 and 360
        angle %= 360;
        //Make angle between -179 and 180
        if (angle > 180) angle -= 360;
        drive.targetAngle = angle;
        timeout.reset();
        rottimer.reset();
    }

    @Override
    public void execute() {

        drive.roterror = getModulusError(angle, drive.getGyroHeading().getDegrees(), -180, 180);

        double output = -drive.roterror * drive.getRotatekP();

        if (Math.abs(output) > Constants.DriveConstants.ROTATE_SPEED) {
            output = Math.signum(output) * Constants.DriveConstants.ROTATE_SPEED;
        }
        this.drive.drive.driveRobotCentric(0, 0, output);

        angleError= drive.targetAngle=-drive.getGyroHeading().getDegrees();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return rottimer.seconds() > 0.1 && Math.abs(angleError) <= 1;

    }

    public static double getModulusError(double reference, double measurement,
                                         double minimumInput,
                                         double maximumInput) {
        double error = reference - measurement;
        double modulus = maximumInput - minimumInput;

        // Wrap error above maximum input
        int numMax = (int) ((error + maximumInput) / modulus);
        error -= numMax * modulus;

        // Wrap error below minimum input
        int numMin = (int) ((error + minimumInput) / modulus);
        error -= numMin * modulus;

        return error;
    }
}

