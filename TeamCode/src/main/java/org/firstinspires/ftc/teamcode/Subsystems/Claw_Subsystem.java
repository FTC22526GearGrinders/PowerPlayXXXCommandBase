package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Claw_Subsystem extends SubsystemBase {


    Servo rightClaw;

    Servo leftClaw;

    private Telemetry telemetry;

    private CommandOpMode myOpMode;


    public Claw_Subsystem(CommandOpMode opMode) {
        myOpMode=opMode;

        rightClaw = myOpMode.  hardwareMap.get(Servo.class, "right claw");
        leftClaw = myOpMode.hardwareMap.get(Servo.class, "left claw");

    }

    @Override

    public void periodic() {

    }

    public void setRightClawPosition(double position) {
        rightClaw.setPosition(position);
    }

    public void setLeftClawPosition(double position) {

        leftClaw.setPosition(position);
    }


    public void close() {
        setRightClawPosition(Constants.ClawConstants.RIGHT_CLAW_CLOSE_POSITION);
        setLeftClawPosition(Constants.ClawConstants.LEFT_CLAW_CLOSE_POSITION);
    }


    public void open() {
        setRightClawPosition(Constants.ClawConstants.RIGHT_CLAW_OPEN_POSITION);
        setLeftClawPosition(Constants.ClawConstants.LEFT_CLAW_OPEN_POSITION);
    }
}
