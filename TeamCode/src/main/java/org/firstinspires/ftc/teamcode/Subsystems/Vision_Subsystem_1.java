package org.firstinspires.ftc.teamcode.Subsystems;


import android.util.Size;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;


public class Vision_Subsystem_1 extends SubsystemBase {

    AprilTagLibrary myAprilTagLibrary;
    AprilTagProcessor myAprilTagProcessor;
// Create the AprilTag processor and assign it to a variable.

    VisionPortal.Builder myVisionPortalBuilder;
    VisionPortal myVisionPortal;

    TfodProcessor myTfodProcessor;

    public CommandOpMode myOpMode;


    public Vision_Subsystem_1(CommandOpMode opMode) {
        myOpMode = opMode;

// Get the AprilTagLibrary for the current season.
        myAprilTagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();


        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(myAprilTagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();


        myTfodProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10)
                .setUseObjectTracker(true)
                .setTrackerMaxOverlap((float) 0.2)
                .setTrackerMinSize(16)
                .build();

        myVisionPortalBuilder = new VisionPortal.Builder();

// Specify the camera to be used for this VisionPortal.
        myVisionPortalBuilder.setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));      // Other choices are: RC phone camera and "switchable camera name".

// Add the AprilTag Processor to the VisionPortal Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // An added Processor is enabled by default.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);       // An added Processor is enabled by default.

// Optional: set other custom features of the VisionPortal (4 are shown here).
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        myVisionPortalBuilder.enableCameraMonitoring(true);      // Enable LiveView (RC preview).
        myVisionPortalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

// Create a VisionPortal by calling build()
        myVisionPortal = myVisionPortalBuilder.build();

    }

    public void initialize() {
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor,false);
        myVisionPortal.setProcessorEnabled(myTfodProcessor,false);

//        myOpMode.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        myOpMode.telemetry.addData("CamNane", visionPortal.getCameraState());
//        myOpMode.telemetry.update();

    }

    @Override

    public void periodic() {
//        myOpMode.telemetry.addData("Image", object);
//        myOpMode.telemetry.addData("result", result);
//        myOpMode.telemetry.addData("bolt0", xy_Bolt[0]);
//        myOpMode.telemetry.addData("bolt1", xy_Bolt[1]);
//        myOpMode.telemetry.addData("targ0", xyTarget[0]);
//        myOpMode.telemetry.addData("targ1", xyTarget[1]);
//        myOpMode.telemetry.update();
    }


}


