package org.firstinspires.ftc.teamcode.Commands.Utils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


public class TFVision extends CommandBase {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #tfod} is the variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private CommandOpMode myOpmode;


    private int test;

    public TFVision(CommandOpMode opMode) {
        myOpmode = opMode;
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                   myOpmode.hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

        // Wait for the DS start button to be touched.

    }

    @Override
    public void initialize() {

        myOpmode.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        myOpmode.telemetry.addData("CamNane",visionPortal.getCameraState());
        myOpmode.telemetry.update();
    }

    @Override
    public void execute() {
        test++;
        myOpmode.telemetry.addData("test", test);
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        myOpmode.telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

//
//            myOpmode.telemetry.addData("", " ");
            myOpmode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            myOpmode.telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            myOpmode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            myOpmode.telemetry.update();
//        }   // end for() loop
        }
    }

    @Override
    public void end(boolean interrupted) {
        visionPortal.close();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
