package org.firstinspires.ftc.teamcode.util.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropLocator {

    public static final int RED_HUE_LOW = -2;
    public static final int RED_HUE_HIGH = 2;

    public static final int BLUE_HUE_LOW = 105;
    public static final int BLUE_HUE_HIGH = 110;
    public enum Location {
        LEFT, MIDDLE, RIGHT
    }

    OpenCvCamera camera;
    PropLocatorPipeline pipeline;
    int low = 0;
    int high = 180;

    public static PropLocator redSideLocator() {
        PropLocator locator = new PropLocator();
        locator.low = RED_HUE_LOW;
        locator.high = RED_HUE_HIGH;
        return locator;
    }

    public static PropLocator blueSideLocator() {
        PropLocator locator = new PropLocator();
        locator.low = BLUE_HUE_LOW;
        locator.high = BLUE_HUE_HIGH;
        return locator;
    }

    public void initializeVision(HardwareMap hw) {
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hw.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDevice();
        pipeline = new PropLocatorPipeline(low, high);
        camera.setPipeline(pipeline);
        stream();
    }

    public void stream() {
        camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(camera, 30);
    }

    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDeviceAsync(() -> {});
    }

    public Location getLocation() {
        return pipeline.currentLocation;
    }

}

@Config
class PropLocatorPipeline extends OpenCvPipeline {

    static final Rect leftROI = new Rect(new Point(0, 0), new Point(100, 100));
    static final Rect midROI = new Rect(new Point(0, 0), new Point(100, 100));
    static final Rect rightROI = new Rect(new Point(0, 0), new Point(100, 100));

    static final Scalar rectColor = new Scalar(255, 255, 255);
    static final Scalar foundColor = new Scalar(0, 255, 0);

    public PropLocator.Location currentLocation;

    int hue_low;
    int hue_high;

    public PropLocatorPipeline(int low, int high) {
        this.hue_low = low;
        this.hue_high = high;
    }

//    public static PropLocatorPipeline redSidePipeline() {
//        return new PropLocatorPipeline(ORANGE_HUE_LOW, ORANGE_HUE_HIGH);
//    }
//
//    public static PropLocator blueSidePipeline() {
//        return new PropLocator(CYAN_HUE_LOW, CYAN_HUE_HIGH);
//    }

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
            currentLocation = PropLocator.Location.LEFT;
        } else if (maxVal == midVal) {
            currentLocation = PropLocator.Location.MIDDLE;
        } else if (maxVal == rightVal) {
            currentLocation = PropLocator.Location.RIGHT;
        }

        if (!showGrayscale) mat = input;
        Imgproc.rectangle(mat, leftROI, (currentLocation == PropLocator.Location.LEFT) ? foundColor : rectColor);
        Imgproc.rectangle(mat, midROI, (currentLocation == PropLocator.Location.MIDDLE) ? foundColor : rectColor);
        Imgproc.rectangle(mat, rightROI, (currentLocation == PropLocator.Location.RIGHT) ? foundColor : rectColor);
        return mat;
    }

    @Override
    public void onViewportTapped() {
        super.onViewportTapped();
        showGrayscale = !showGrayscale;
    }
}
