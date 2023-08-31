package org.firstinspires.ftc.teamcode.OpCodes_Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.teamcode.Commands.Elevator.PositionElevator;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator_Subsystem;

/**
 * Created by Tom on 9/26/17.  Updated 9/24/2021 for PIDF.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller.  This OpMode uses the extended/enhanced
 * PIDF-related functions of the DcMotorEx class.  The REV Robotics Expansion Hub
 * supports the extended motor functions, but other controllers (such as the
 * deprecated Modern Robotics and Hitechnic DC Motor Controllers) do not.
 */
@Config
@Autonomous(name = "Elevator: Tune PID", group = "Tune")

public class TuneElevatorPositionPID extends CommandOpMode {

    // our DC motor
    Elevator_Subsystem elevator = new Elevator_Subsystem(this);

    public static PIDCoefficients elTunePID = new PIDCoefficients(0, 0, 0);


    FtcDashboard dashboard;

    public static double TARGET_POSITION = 100;

    double integral = 0;

    public void initialize() {

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

        // Put run blocks here.
        public void run() {


            if (TARGET_POSITION != elevator.targetPosition)
                new PositionElevator(elevator, TARGET_POSITION).schedule();
            if (elTunePID.p != elevator.getPositionKp())
                elevator.setPositionKp(elTunePID.p);





        telemetry.addData("power", elevator.getPower());
        telemetry.addData("pos curr", elevator.getPosition());
        telemetry.addData("position", elevator.targetPosition);
        telemetry.addData("p", elTunePID.p);
        telemetry.addData("d", elTunePID.d);
        telemetry.addData("TP", TARGET_POSITION);

        telemetry.update();


    }
}
