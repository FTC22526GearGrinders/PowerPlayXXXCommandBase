package org.firstinspires.ftc.teamcode.Components.CV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
/*@Config*/
public class SpikeTapeObserverPipeline extends OpenCvPipeline {

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;

    Rect currentCrop;

    public SpikeTapeObserverPipeline(Rect currentCrop) {
        this.currentCrop = currentCrop;
        frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat uncropped = input;

        Mat cropped = new Mat(uncropped, currentCrop);

        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(cropped, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        Mat lower_red_hue_range = new Mat();
        Mat upper_red_hue_range = new Mat();
        Scalar a = new Scalar(0, 100, 100);
        Scalar b = new Scalar(10, 255, 255);
        Scalar c = new Scalar(160, 100, 100);
        Scalar d = new Scalar(179, 255, 255);

        Core.inRange(cropped, a, b, lower_red_hue_range);

        Core.inRange(cropped, c, d, upper_red_hue_range);


        Mat red_hue_masked = new Mat();

        Core.addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_masked);

        List<MatOfPoint> contours = new ArrayList<>();

        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges

        Imgproc.findContours(red_hue_masked, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        List<MatOfPoint> contoursa = new ArrayList<MatOfPoint>();
        Imgproc.findContours(red_hue_masked, contoursa, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = -1;
        int maxAreaIdx = -1;
        for (int idx = 0; idx < contours.size(); idx++) {
            Mat contour = contours.get(idx);
            double contourarea = Imgproc.contourArea(contour);
            if (contourarea > maxArea) {
                maxArea = contourarea;
                maxAreaIdx = idx;
            }
        }

//        MatOfPoint2f approxCurve = new MatOfPoint2f();
//        //For each contour found
//        for (int i = 0; i < contours.size(); i++) {
//            //Convert contours(i) from MatOfPoint to MatOfPoint2f
//            MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
//            //Processing on mMOP2f1 which is in type MatOfPoint2f
//            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
//
//            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);
//
//            //Convert back to MatOfPoint
//            MatOfPoint points = new MatOfPoint(approxCurve.toArray());
//
//            // Get bounding rect of contour
//            Rect rect = Imgproc.boundingRect(points);
//
//            // draw enclosing rectangle (all same color, but you could use variable i to make them unique)
//
//            new Point(rect.x + rect.width, rect.y + rect.height);
//            new Scalar(255, 0, 0);
        //    }

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }


        //release all the data
        // input.release();

        mat.release();


        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        return input;
    }

    //return contour area to look command
    public int getArea() {
        return 911;
    }
}