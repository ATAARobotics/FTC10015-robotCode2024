package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RedCubePipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);
        Core.inRange(
                input,
                // red airplane paper
                new Scalar(120, 120, 120),
                new Scalar(160, 180, 160),
                input
        );

        return input;
    }
}
