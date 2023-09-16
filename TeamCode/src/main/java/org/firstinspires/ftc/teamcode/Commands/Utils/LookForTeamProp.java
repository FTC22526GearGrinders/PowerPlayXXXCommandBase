package org.firstinspires.ftc.teamcode.Commands.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.CV.SpikeTapeObserverPipeline;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*Camera needs to see all 3 spike tapes with enough space to the left, top and right for
 *the area of the prop
 * Tape is 1" wide x 13" long while prop is 3" x 3" x 3"
 * Tiles are nominal 24" square.
 *  Distance between left and right tape centers is nominal 22"
 * Gap left and right at top tap approx. 5"
 * Allow 6" left, right and center means camera view is 34" wide x 18"high
 * Camera resolution is 640 px wide  x320 px high px count starts top left corner x left y down
 *Px per inch = 640/34 approx 20 width and 320/18 approx 18 high
 *Spike tape pixels = 20 * 12 + 18 *1 = 258
 *
 * */
public class LookForTeamProp extends CommandBase {
    private OpenCvWebcam webcam;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;

    int lx = 0;//x start point of crop pixels left edge screen
    int ly = 108;//y start point of crop 6" down
    int lh = 252;//height of crop say 14"
    int lw = 160;//width of crop say 8"
    Rect leftMarkCrop = new Rect(lx, ly, lw, lh);

    int tx = 0;//10""
    int ty = 0;// 5"
    int th = 0;//8"
    int tw = 0;//14"
    Rect topMarkCrop = new Rect(tx, ty, tw, th);

    int rx = 0;//20"
    int ry = 0;// 5"
    int rh = 0; //8""
    int rw = 0;// 14""
    Rect rightMarkCrop = new Rect(rx, ry, rw, rh);

    Rect currentCrop = leftMarkCrop;

    int leftAreaPxNoProp = 258;//will be less because of camera angle

    int topAreaPxNoProp = 258;

    int rightAreaPxNoProp = 258;

    int currentPXNoProp = 0;

    int spikeMark;

    SpikeTapeObserverPipeline sptop = null;

    CommandOpMode myOpMode;

    FtcDashboard dashboard;
    private final int propSeenArea = 500;
    private boolean propSeen;

    private boolean tapeSeen;
    private int tapeSeenArea;

    public LookForTeamProp(CommandOpMode opMode, int spikeMark) {
        myOpMode = opMode;
        this.spikeMark = spikeMark;
    }

    @Override
    public void initialize() {

        currentPXNoProp = 0;

        propSeen = false;

        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, dashboard.getTelemetry());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));

        switch (spikeMark) {

            case 0:
                currentCrop = leftMarkCrop;
                currentPXNoProp = leftAreaPxNoProp;
                break;
            case 1:
                currentCrop = topMarkCrop;
                currentPXNoProp = topAreaPxNoProp;
                break;
            case 2:
                currentCrop = rightMarkCrop;
                currentPXNoProp = rightAreaPxNoProp;
                break;
            default:
                break;

        }

        webcam.startStreaming(RESOLUTION_WIDTH, RESOLUTION_HEIGHT);

        sptop = new SpikeTapeObserverPipeline(currentCrop);

        webcam.setPipeline(sptop);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                //start streaming the camera
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                //if you are using dashboard, update dashboard camera view
                dashboard.startCameraStream(webcam, 5);

            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


    }

    @Override
    public void execute() {
        int currentArea = sptop.getArea();

        if(currentArea > tapeSeenArea)
            tapeSeen = true;

        if (currentArea > propSeenArea)
            propSeen = true;

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
