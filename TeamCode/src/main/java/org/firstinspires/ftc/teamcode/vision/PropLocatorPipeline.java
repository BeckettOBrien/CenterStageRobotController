package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropLocatorPipeline extends OpenCvPipeline {

    static final Rect leftROI = new Rect(new Point(0, 0), new Point(100, 100));
    static final Rect midROI = new Rect(new Point(0, 0), new Point(100, 100));
    static final Rect rightROI = new Rect(new Point(0, 0), new Point(100, 100));

    public static final int ORANGE_HUE_LOW = 15;
    public static final int ORANGE_HUE_HIGH = 20;

    public static final int CYAN_HUE_LOW = 90;
    public static final int CYAN_HUE_HIGH = 95;

    static final Scalar rectColor = new Scalar(255, 255, 255);
    static final Scalar foundColor = new Scalar(0, 255, 0);

    public enum Location {
        LEFT, MIDDLE, RIGHT
    }

    public Location currentLocation;

    int hue_low;
    int hue_high;

    public PropLocatorPipeline(int low, int high) {
        this.hue_low = low;
        this.hue_high = high;
    }

    public static PropLocatorPipeline redSidePipeline() {
        return new PropLocatorPipeline(ORANGE_HUE_LOW, ORANGE_HUE_HIGH);
    }

    public static PropLocatorPipeline blueSidePipeline() {
        return new PropLocatorPipeline(CYAN_HUE_LOW, CYAN_HUE_HIGH);
    }

    Mat mat = new Mat();

    public static boolean showGrayscale = false;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, new Scalar(hue_low, 100, 100), new Scalar(hue_high, 255, 255), mat);

        Mat left = mat.submat(leftROI);
        double leftVal = Core.mean(left).val[0];
        left.release();

        Mat mid = mat.submat(midROI);
        double midVal = Core.mean(mid).val[0];
        mid.release();

        Mat right = mat.submat(rightROI);
        double rightVal = Core.mean(right).val[0];
        right.release();

        double maxVal = Math.max(Math.max(leftVal, midVal), rightVal);
        if (maxVal == leftVal) {
            currentLocation = Location.LEFT;
        } else if (maxVal == midVal) {
            currentLocation = Location.MIDDLE;
        } else if (maxVal == rightVal) {
            currentLocation = Location.RIGHT;
        }

        if (!showGrayscale) mat = input;
        Imgproc.rectangle(mat, leftROI, (currentLocation == Location.LEFT) ? foundColor : rectColor);
        Imgproc.rectangle(mat, midROI, (currentLocation == Location.MIDDLE) ? foundColor : rectColor);
        Imgproc.rectangle(mat, rightROI, (currentLocation == Location.RIGHT) ? foundColor : rectColor);
        return mat;
    }

    @Override
    public void onViewportTapped() {
        super.onViewportTapped();
        showGrayscale = !showGrayscale;
    }
}
