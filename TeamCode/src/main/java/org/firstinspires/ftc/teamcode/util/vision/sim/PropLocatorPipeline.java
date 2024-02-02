package org.firstinspires.ftc.teamcode.util.vision.sim;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropLocatorPipeline extends OpenCvPipeline {

    Mat mat = new Mat();
    Mat temp = new Mat();
    Mat temp2 = new Mat();

    @Override
    public Mat processFrame(Mat input) {
//        Mat out = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, new Scalar(-2, 100, 100), new Scalar(2, 255, 255), temp);
        Imgproc.cvtColor(temp, temp2, Imgproc.COLOR_GRAY2RGB);
        Core.bitwise_and(mat, temp2, mat);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);
        return mat;
    }

}
