package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.threeten.bp.DayOfWeek;

import java.util.List;

public class Vision_Subsystems extends SubsystemBase {
    private static final boolean USE_WEBCAM = true;
    public TfodProcessor tfod;
    private VisionPortal visionPortal;

    private Telemetry telemetry;

    private CommandOpMode myOpMode;

    private HardwareMap hardwareMap;
    private Recognition[] currentRecognitions;


    public Vision_Subsystems(CommandOpMode opMode) {
        myOpMode = opMode;
        initTfod();

    }

    @Override

    public void periodic() {



    }

    public void initTfod() {
        tfod = TfodProcessor.easyCreateWithDefaults();
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }
    }





    public void findObjects() {



        List<Recognition> currentRecognitions = tfod.getRecognitions();
        myOpMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
        myOpMode.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        myOpMode.telemetry.addData(">", "Touch Play to start OpMode");
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            myOpMode.telemetry.addData(""," ");
            myOpMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            myOpMode.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            myOpMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            myOpMode.telemetry.update();
        }   // end for() loop

    }   // end method telemetryTfod()
}
