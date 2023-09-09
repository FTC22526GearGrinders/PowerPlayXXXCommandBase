package org.firstinspires.ftc.teamcode.OpCodes_Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.CVMasterSubsystem;

@Autonomous
public class FindPoleJunction extends CommandOpMode {

    private CVMasterSubsystem cvm;

    @Override
    public void initialize() {cvm = new CVMasterSubsystem(this);
    }


    @Override
    public void run() {

        //cvm.execute();

        CommandScheduler.getInstance().run();
    }
}
