package org.firstinspires.ftc.teamcode;

import org.apache.commons.math3.ml.neuralnet.twod.util.TopographicErrorHistogram;
import org.apache.commons.math3.stat.inference.AlternativeHypothesis;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FindProp extends OpenCvPipeline {
    public int circleNum = 0;
    public Point recentCircle = new Point(-1, 0);
    public double circleRadius = 0;

    @Override
    public Mat processFrame(Mat input) {
        if (input.empty()) {
            return input;
        }
        Mat grey = new Mat();
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2HSV);
        Core.inRange(grey, new Scalar(0, 60, 80), new Scalar(180, 255, 255), grey);
        /* Denoises image by applying a slight blur */
        Core.bitwise_not(grey, grey);

        input.setTo(new Scalar(0, 0, 0), grey);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);
        Imgproc.medianBlur(input, input, 5);
        Mat circles = new Mat();

        /* Uses Hough model fitting algorithm to find circles in the image */
        Imgproc.HoughCircles(input, circles, Imgproc.HOUGH_GRADIENT, 1, 45, 40, 20, 1, 75);

        /* renders the circles to the stream image */
        circleNum = circles.cols();

        for (int i = 0; i < circles.cols(); i++) {
            double[] circle = circles.get(0, i);
            Point center = new Point(circle[0], circle[1]);
            double radius = circle[2];
            if (radius > 35) {
                circleRadius = radius;
                recentCircle = center;
                Imgproc.circle(input, center, (int)radius, new Scalar(255, 0, 255));
            }
        }
        return input;
    }
}