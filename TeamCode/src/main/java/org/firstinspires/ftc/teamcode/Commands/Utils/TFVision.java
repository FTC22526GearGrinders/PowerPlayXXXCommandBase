package org.firstinspires.ftc.teamcode.Commands.Utils;

import android.os.DropBoxManager;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Subsystems.Drive_Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision_Subsystems;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


public class TFVision extends CommandBase {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private Vision_Subsystems vision_subsystems;



    public Drive_Subsystem drive;

    public Vision_Subsystems vision;


    /**
     * {@link #tfod} is the variable to store our instance of the TensorFlow Object Detection processor.
     */


    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;






    private int test;



    public TFVision(Drive_Subsystem drive,Vision_Subsystems vision) {
        this.drive = drive;
        this.vision = vision;



        // Create the vision portal the easy way.


        // Wait for the DS start button to be touched.

    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {

       vision. result = getConeImage();


        if (vision.result == 1)vision.xyTarget=vision.xy_Bolt.clone();

        if (vision.result == 2)vision.xyTarget=vision.xy_Bulb.clone();

        if (vision.result == 3)vision.xyTarget=vision.xy_Panel.clone();



    }

    public int getConeImage() {
        int temp = 0;
        vision.object="";
        List<Recognition> currentRecognitions = vision.tfod.getRecognitions();
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
           vision .object = recognition.getLabel();


        }

        if (vision.object == "1 Bolt")
            temp = 1;
        if (vision.object == "2 Bulb")
            temp = 2;
        if (vision.object == "3 Panel")
            temp = 3;

        return temp;
    }




    @Override
    public void end(boolean interrupted) {
        visionPortal.close();
    }

    @Override
    public boolean isFinished() {
        return false;//result!=0;
    }
}
