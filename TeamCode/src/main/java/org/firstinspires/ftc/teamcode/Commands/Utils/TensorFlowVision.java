//package org.firstinspires.ftc.teamcode.Commands.Utils;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Subsystems.Vision_Subsystems;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//
//public class TensorFlowVision extends CommandBase {
//
//private Vision_Subsystems vision_subsystems;
//public TensorFlowVision(Vision_Subsystems vision_subsystems) {
//    this.vision_subsystems = vision_subsystems;
//}
//
//    @Override
//    public void initialize() {
//
//
//    }
//
//    @Override
//    public void execute() {
//
//vision_subsystems.findObjects();
//
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//
//    }
//
//    @Override
//    public boolean isFinished() {
//        return false;
//    }
//}
