package org.firstinspires.ftc.teamcode.OpCodes_Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Drive.PositionDrive;
import org.firstinspires.ftc.teamcode.Logging.Datalogger;
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
@Autonomous(name = "Drive: Tune Posn PID", group = "Tune")

public class TuneDrivePositionPID extends CommandOpMode {

    // our DC motor
    Drive_Subsystem drive;
    public static PIDCoefficients drTunePID = new PIDCoefficients(0, 0, 0);

    ElapsedTime PIDTimer = new ElapsedTime();

    FtcDashboard dashboard;

    boolean doLogging = false;

    Datalog datalog;

    public static double TARGET_POSITION = 100;


    @Override
    public void initialize() {

        drive = new Drive_Subsystem(this);

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize the datalog

        if (doLogging) {

            datalog = new Datalog("TuneDrivePosn-01");
//
            datalog.opModeStatus.set("RUNNING");
        }

    }


    public void run() {

        if (TARGET_POSITION != drive.targetDistance)
            new PositionDrive(drive, TARGET_POSITION).withTimeout(10000).schedule();
        if (drTunePID.p != drive.getPositionKp())
            drive.setPositionKp(drTunePID.p);

        if(doLogging) {

            datalog.target.set(TARGET_POSITION);
            datalog.position.set(drive.getAverageFrontDistance());
            datalog.error.set(TARGET_POSITION - drive.getAverageFrontDistance());
            datalog.output.set(drive.getPower());
            datalog.pgain.set(drive.getPositionKp());
        }

        telemetry.addData("error", TARGET_POSITION - drive.getAverageFrontDistance());
        telemetry.addData("pos curr", drive.getAverageFrontDistance());
        telemetry.addData("position", TARGET_POSITION);
        telemetry.addData("p", drTunePID.p);
        telemetry.addData("d", drTunePID.d);
        telemetry.addData("TP", TARGET_POSITION);

        telemetry.update();

    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");

        public Datalogger.GenericField target = new Datalogger.GenericField("Target");

        public Datalogger.GenericField position = new Datalogger.GenericField("Position");

        public Datalogger.GenericField error = new Datalogger.GenericField("Error");

        public Datalogger.GenericField output = new Datalogger.GenericField("Output");

        public Datalogger.GenericField pgain = new Datalogger.GenericField("Pgain");


        public Datalog(String name) {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            target,
                            position,
                            error,
                            output,
                            pgain
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }

}


