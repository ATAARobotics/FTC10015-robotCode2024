package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ReverseTeamElementPipeline extends OpenCvPipeline {
    public enum Result {Unknown, Left, Middle, Right};
    public Result result = Result.Unknown;
    Scalar min;
    Scalar max;
    public Mat processed;
    public Mat annotated;
    // for blue side only
    int x0 = 130;
    int y0 = 255;
    int x1 = 469;
    int y1 = 142;
    int x2 = 560;
    int y2 = 247;


    public ReverseTeamElementPipeline() {
        if (true) {
            // march 12, 2024:final values for close-side red start
            // RED side 920s webcam
            min = new Scalar(100, 100, 100);
            max = new Scalar(160, 255, 255);
            x0 = 87;
            y0 = 360;
            x1 = 425;
            y1 = 355;
            x2 = 0;
            y2 = 0;
        } else {
            // march 12, 2024:values for far-side red start
            // (which should be also for blue-side CLOSE start)
            // blue side, 920s camera
            min = new Scalar(10, 31, 69);
            max = new Scalar(69, 255, 255);
            x0 = 0;
            y0 = 0;
            x1 = 358;
            y1 = 340;
            x2 = 645;
            y2 = 360;
        }
        processed = new Mat();
        annotated = new Mat();
    }
    public ReverseTeamElementPipeline(boolean red, boolean close) {
        // XXX going to want to do "near/far" AND "red/blue" probably?
        if (red) {
            min = new Scalar(100, 100, 100);
            max = new Scalar(160, 255, 255);
        } else {
            min = new Scalar(10, 31, 69);
            max = new Scalar(69, 255, 255);
        }
        if (red) {
            if (close) {
                // close-side red
                x0 = 87;
                y0 = 360;
                x1 = 425;
                y1 = 355;
                x2 = 0;
                y2 = 0;
            } else {
                x0 = 0;
                y0 = 0;
                x1 = 358;
                y1 = 340;
                x2 = 645;
                y2 = 360;
            }
        } else {
            if (close) {
                // XXX same as far-side red!
                x0 = 0;
                y0 = 0;
                x1 = 358;
                y1 = 340;
                x2 = 645;
                y2 = 360;
            } else {
                // XXX same as far-side blue!
                x0 = 87;
                y0 = 360;
                x1 = 425;
                y1 = 355;
                x2 = 0;
                y2 = 0;
            }
        }
        processed = new Mat();
        annotated = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {
        // don't create new Mat's in this method? apparently ... Mat processed = new Mat();
        Imgproc.cvtColor(input, processed, Imgproc.COLOR_BGR2HSV);

        // filter for one colour
        Core.inRange(processed, min, max, processed);

        //Imgproc.dilate(processed, processed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        //Imgproc.dilate(processed, processed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        //Imgproc.dilate(processed, processed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

        int width = 50;
        int height = 55;



        // count how many pixels are "on" in each region of interest

        int left = 0;
        for (int x=x0; x < x0 + width; x++) {
            for (int y=y0; y < y0 + height; y++) {
                double[] px = processed.get(y, x);
                if (px != null && (int)px[0] > 128) {
                    left++;
                }
            }
        }

        int mid = 0;
        for (int x=x1; x < x1 + width; x++) {
            for (int y=y1; y < y1 + height; y++) {
                double[] px = processed.get(y, x);
                if (px != null && (int)px[0] > 128){
                    mid++;
                }
            }
        }

        int right = 0;
        for (int x=x2; x < x2 + width; x++) {
            for (int y=y2; y < y2 + height; y++) {
                double[] px = processed.get(y, x);
                if (px != null && (int)px[0] > 128) {
                    right++;
                }
            }
        }


        Scalar red = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0, 255, 0);
        int biggest = Math.max(right, Math.max(left, mid));
        if (biggest > 100) {
            if (left == biggest) { result = Result.Left;  }
            if (mid == biggest) { result = Result.Middle; }
            if (right == biggest) { result = Result.Right; }
        } else {
            if (x0 == 0) {
                result = Result.Left;
            } else {
                result = Result.Right;
            }
        }

        if (true) {
            input.copyTo(annotated);
            Imgproc.putText(annotated, "L:" + (left / (50.0*55)), new Point(x0, y0), Imgproc.FONT_HERSHEY_PLAIN, 1, red);
            Imgproc.putText(annotated, "M:" + (mid / (50.0*55)), new Point(x1, y1), Imgproc.FONT_HERSHEY_PLAIN, 1, red);
            Imgproc.putText(annotated, "R:" + biggest, new Point(x2, y2), Imgproc.FONT_HERSHEY_PLAIN, 1, red);

            Imgproc.rectangle(annotated, new Point(x0, y0), new Point(x0 + width, y0 + width), result == Result.Left ? green : red, 2);
            Imgproc.rectangle(annotated, new Point(x1, y1), new Point(x1 + width, y1 + width), result == Result.Middle ? green : red, 2);
            Imgproc.rectangle(annotated, new Point(x2, y2), new Point(x2 + width, y2 + width), result == Result.Right ? green : red, 2);
            return annotated;
        }
        return processed;
    }
}
