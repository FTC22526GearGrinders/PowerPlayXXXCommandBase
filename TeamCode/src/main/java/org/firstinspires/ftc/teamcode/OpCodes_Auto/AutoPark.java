package org.firstinspires.ftc.teamcode.OpCodes_Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Drive.PositionDrive;
import org.firstinspires.ftc.teamcode.Commands.Utils.TFVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drive_Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision_Subsystems;


@Autonomous
public class AutoPark extends CommandOpMode {

    protected Drive_Subsystem drive;
    protected Vision_Subsystems vision;


    @Override
    public void initialize() {
        drive = new Drive_Subsystem(this);
        vision = new Vision_Subsystems(this);
        double x;
        double y;
        register(drive,vision);
;
        new TFVision(this.drive,this.vision)
//        new
//
//                SequentialCommandGroup(
//
//
//                new PositionDrive(drive, drive.xyTarget[0])
        .schedule();

    }


    @Override
    public void run() {


        CommandScheduler.getInstance().run();
    }
}
