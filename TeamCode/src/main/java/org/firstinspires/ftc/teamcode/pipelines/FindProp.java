package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FindProp extends OpenCvPipeline {


    @Override
    public Mat processFrame(Mat input) {
        Mat blur = new Mat();

        /* Denoises image by applying a slight blur */
        Imgproc.medianBlur(input, blur, 5);
        Mat circles = new Mat();

        /* Uses Hough model fitting algorithm to find circles in the image */
        Imgproc.HoughCircles(input, circles, Imgproc.HOUGH_GRADIENT, 1, 45);

        /* renders the circles to the stream image */
        for (int i = 0; i < circles.cols(); i++) {
            double[] circle = circles.get(0, i);
            Point center = new Point(circle[0], circle[1]);
            double radius = circle[2];

            Imgproc.circle(blur, center, (int)radius, new Scalar(255, 0, 255));
        }


        return blur;
    }
}
