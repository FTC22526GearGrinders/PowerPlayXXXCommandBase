package org.firstinspires.ftc.teamcode.OpCodes_Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Drive.RotateToAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Drive_Subsystem;


/**
 * Created by Tom on 9/26/17.  Updated 9/24/2021 for PIDF.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller.  This OpMode uses the extended/enhanced
 * PIDF-related functions of the DcMotorEx class.  The REV Robotics Expansion Hub
 * supports the extended motor functions, but other controllers (such as the
 * deprecated Modern Robotics and Hitechnic DC Motor Controllers) do not.
 */
@Config
@Autonomous(name = "Drive: Tune Rotate PID", group = "Tune")

public class TuneDriveRotateP extends CommandOpMode {

    // our DC motor
    Drive_Subsystem drive;
    public static PIDCoefficients drTunePID = new PIDCoefficients(0, 0, 0);

    ElapsedTime PIDTimer = new ElapsedTime();

    FtcDashboard dashboard;

    public static double TARGET_POSITION = 100;


    @Override
    public void initialize() {

        drive = new Drive_Subsystem(this);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(6);

    }

    // Put run blocks here.
    public void run() {


        if (TARGET_POSITION != drive.targetDistance)
            new RotateToAngle(drive, TARGET_POSITION).withTimeout(10000).schedule();
        if (drTunePID.p != drive.getRotatekP())
            drive.setRotatekP(drTunePID.p);


        telemetry.addData("error", TARGET_POSITION - drive.getGyroHeading().getDegrees());

        telemetry.addData("position", TARGET_POSITION);
        telemetry.addData("p", drTunePID.p);
        telemetry.addData("d", drTunePID.d);
        telemetry.addData("TP", TARGET_POSITION);

        telemetry.update();

    }

}


