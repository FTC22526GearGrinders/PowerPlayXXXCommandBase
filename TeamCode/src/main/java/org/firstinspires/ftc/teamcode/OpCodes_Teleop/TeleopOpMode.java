package org.firstinspires.ftc.teamcode.OpCodes_Teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.Drive.JogDrive;
import org.firstinspires.ftc.teamcode.Commands.Drive.PositionDrive;
import org.firstinspires.ftc.teamcode.Commands.Drive.StrafeDrive;
import org.firstinspires.ftc.teamcode.Commands.Elevator.PositionElevator;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive_Subsystem;

@TeleOp
public class TeleopOpMode extends CommandOpMode {

    protected Drive_Subsystem drive;
   //protected Vision_Subsystems vision_subsystems;
    protected Elevator_Subsystem elevator;
    protected Claw_Subsystem claw;

     HardwareMap hardwareMap;

    protected GamepadEx driverGamepad;
    protected GamepadEx secondaryGamepad;


    @Override
    public void initialize() {
        drive = new Drive_Subsystem(this);

        //vision_subsystems = new Vision_Subsystems(this);

        elevator = new Elevator_Subsystem(this);

        claw = new Claw_Subsystem(this);

//new TensorFlowVision(vision_subsystems).schedule();
        // register(drive, elevator, claw);

        driverGamepad = new GamepadEx(gamepad1);

        secondaryGamepad = new GamepadEx(gamepad2);


//        drive.setDefaultCommand(new JogDrive(this.drive, driverGamepad));
//
//        elevator.setDefaultCommand(new HoldElevatorAtPosition(this.elevator));


        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                // .whenPressed(new CloseClaw(this));
                .whenPressed(new InstantCommand(claw::close, claw));

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(claw::open, claw));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(new JogElevatorPlus(this.elevator));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(new JogElevatorMinus(this.elevator));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new PositionElevator(this.elevator, Constants.ElevatorConstants.LOW_JUNCTION).withTimeout(10000));

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new PositionElevator(this.elevator, Constants.ElevatorConstants.MID_JUNCTION).withTimeout(10000));

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new PositionElevator(this.elevator, Constants.ElevatorConstants.HIGH_JUNCTION).withTimeout(10000));

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new PositionElevator(this.elevator, Constants.ElevatorConstants.GROWND_JUNCTION).withTimeout(10000));

        driverGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(drive::toggleFieldOrient));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new PositionDrive(this.drive, 20).withTimeout(10000));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new PositionDrive(this.drive, 30).withTimeout(10000));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new PositionDrive(this.drive, -20).withTimeout(10000));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new PositionDrive(this.drive, 0).withTimeout(10000));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new RotateToAngle(this.drive, 90).withTimeout(10000));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new RotateToAngle(this.drive, -90).withTimeout(10000));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new RotateToAngle(this.drive, 179).withTimeout(10000));


        secondaryGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new RotateToAngle(this.drive, -45).withTimeout(10000));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new StrafeDrive(this.drive,10).withTimeout(10000));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new StrafeDrive(this.drive,5).withTimeout(10000));


        secondaryGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(drive::reset, drive));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new TFVision(this));

        register(drive, elevator, claw);
        drive.setDefaultCommand(new JogDrive(this.drive, driverGamepad));

        elevator.setDefaultCommand(new HoldElevatorAtPosition(this.elevator));
    }


    public void run() {

        CommandScheduler.getInstance().run();

        //elevator.showTelemetry();
        drive.showTelemetry();
    }


}