package org.firstinspires.ftc.teamcode.OpCodes_Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Drive.PositionDrive;
import org.firstinspires.ftc.teamcode.Commands.Drive.RotateToAngle;
import org.firstinspires.ftc.teamcode.Commands.Elevator.HoldElevatorAtPosition;
import org.firstinspires.ftc.teamcode.Commands.Elevator.PositionElevator;
import org.firstinspires.ftc.teamcode.Commands.Utils.TimeDelay;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Claw_Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive_Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator_Subsystem;

@Autonomous

public class OneConeDeliverOpMode extends CommandOpMode {

    protected Drive_Subsystem drive;
    protected Elevator_Subsystem elevator;
    protected Claw_Subsystem claw;


    @Override
    public void initialize() {

        drive = new Drive_Subsystem(this);

        elevator = new Elevator_Subsystem(this);

        claw = new Claw_Subsystem(this);

        register(drive, elevator, claw);

        elevator.setDefaultCommand(new HoldElevatorAtPosition(elevator));


        new SequentialCommandGroup(

                 new InstantCommand(claw::close, claw),
                new TimeDelay(0.5),

                new PositionDrive(this.drive, 20).withTimeout(3000),

                new TimeDelay(2.5),


                new RotateToAngle(this.drive, 90).withTimeout(10000),

                //new InstantCommand(this.drive::reset),

                new PositionElevator(this.elevator, Constants.ElevatorConstants.HIGH_JUNCTION).withTimeout(5000),

              //  new PositionDrive(this.drive, 30).withTimeout(10000),

                new InstantCommand(claw::open, claw),

                new TimeDelay(0.5),

                new PositionElevator(this.elevator, Constants.ElevatorConstants.LOWER_POSITION_LIMIT)

               // new PositionDrive(this.drive, 5).withTimeout(10000)




        ).schedule();

    }


    @Override
    public void run() {


        CommandScheduler.getInstance().run();
    }
}