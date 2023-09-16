package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


public class Vision_Subsystems extends SubsystemBase {
    public int result;
    public String object;
    private VisionPortal visionPortal;


    private double Lablel;

    private double x;
    private double y;


    public CommandOpMode myOpMode;
    public TfodProcessor tfod;

    public double[] xy_Bolt = {5, 3};

    public double[] xy_Bulb = {5, 4};

    public double[] xy_Panel = {5, 5};

    public double[] xyTarget = {0, 0};


    boolean USE_WEBCAM = true;


    public Vision_Subsystems(CommandOpMode opMode) {
        myOpMode = opMode;
        tfod = TfodProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }

    public void initialize() {
        myOpMode.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        myOpMode.telemetry.addData("CamNane", visionPortal.getCameraState());
        myOpMode.telemetry.update();

    }

    @Override

    public void periodic() {
        myOpMode.telemetry.addData("Image", object);
        myOpMode.telemetry.addData("result", result);
        myOpMode.telemetry.addData("bolt0", xy_Bolt[0]);
        myOpMode.telemetry.addData("bolt1", xy_Bolt[1]);
        myOpMode.telemetry.addData("targ0", xyTarget[0]);
        myOpMode.telemetry.addData("targ1", xyTarget[1]);
        myOpMode.telemetry.update();
    }


}


