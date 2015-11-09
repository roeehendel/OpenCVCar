package com.roee.opencv.opencv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class LaneDetector {

    private double VerticalThresholdSlope = 4;

    private Mat mRgba;
    private Mat mLines;
    private Mat mFrameWithLanes;
    private Mat mTemp;

    private double[][] mLanes;
    private double[] mBisectorLine;

    private int mVerticalLineCount;

    public LaneDetector(){
        mRgba = new Mat();
        mLines = new Mat();
        mLanes = new double[3][2];
        mBisectorLine = new double[2];
        mFrameWithLanes = new Mat();
        mTemp = new Mat();
    }

    public void proccessFrame(Mat frame){

        mVerticalLineCount = 0;

        mRgba = frame;

        // Detect all lines in image
        detectLines();
        // Process lines to extract lanes
        extractLanes();

        calculateAngleBisector();
    }

    public void detectLines(){

        // Convert image to grayscale
        Imgproc.cvtColor(mRgba, mTemp, Imgproc.COLOR_RGB2GRAY);

        double otsu_thresh_val = Imgproc.threshold(
                mTemp, new Mat(), 0, 255, Imgproc.THRESH_OTSU | Imgproc.THRESH_BINARY);

        // Apply Canny to image
        double threshold1 = otsu_thresh_val/2;
        double threshold2 = otsu_thresh_val;
        Imgproc.Canny(mTemp, mTemp, threshold1, threshold2);

//        Imgproc.pyrUp(mTemp, mTemp);

        // Detect lines with Hough Transform
        Imgproc.HoughLinesP(mTemp, mLines, 1, Math.PI / 180, 100, 150, 70);

        // Release unnecessary temporary Mat
//        mTemp.release();

    }

    public void extractLanes(){

        mLanes = new double[3][2];

        double lastB = Double.POSITIVE_INFINITY;

        int row = 0;
        int x = 0;

        while ( x < mLines.rows() && row < mLanes.length - 1) {

            double[] vec = mLines.get(x, 0);
            double y1 = vec[0],
                    x1 = vec[1],
                    y2 = vec[2],
                    x2 = vec[3];

            double length = Math.sqrt(Math.pow(y1-y2, 2) + Math.pow(x1-x2, 2));

            // Decide whether a line qualifies as a lane or not
            if(length > DrivingActivity.mFrameHeight / 64) {

                // Extract the line's Cartesian properties (y = a*x + b)
                double a = (y1 - y2) / (x1 - x2);

                double b = y1 - a * x1;

                if (Math.abs(b - lastB) > 100 && Math.abs(a) < VerticalThresholdSlope) {
                    mLanes[row] = new double[]{a, b};
                    mVerticalLineCount++;
                    lastB = b;
                    row++;
                }
            }
            x++;
        }

        double[] lane1 = mLanes[0];
        double[] lane2 = mLanes[1];

        if(lane1[0] > 0){
            mLanes[0] = lane1;
            mLanes[1] = lane2;
        }else{
            mLanes[0] = lane2;
            mLanes[1] = lane1;
        }

    }

    public void createFrameWithLanes(int[] lanesColor){
        mRgba.copyTo(mFrameWithLanes);

        for (int x = 0; x < mLanes.length; x++) {
            // Get line y = a*x + b. [0] = a, [1] = b
            double[] line = mLanes[x];

            double a = line[0],
                    b = line[1],
                    y1 = 0,
                    y2 = mRgba.width(),
                    x1 = a * y1 + b,
                    x2 = a * y2 + b;

            if(a != 0 && b != 0){

                Point start = new Point(x1, y1);
                Point end = new Point(x2, y2);

                Imgproc.line(mFrameWithLanes,
                        start, end,
                        new Scalar(x * lanesColor[0],x * lanesColor[1] ,x *lanesColor[2]),
                        4);
            }
        }
    }

    public void calculateAngleBisector(){
        double[] lane1 = mLanes[0],
                 lane2 = mLanes[1];

        double a1 = lane1[0],
                b1 = lane1[1],
                a2 = lane2[0],
                b2 = lane2[1],
                A1 = -a1,
                A2 = - a2,
                B12 = 1,
                C1 = -b1,
                C2 = -b2,
                R1 = Math.sqrt(A1 * A1 + 1),
                R2 = Math.sqrt(A2 * A2 + 1),
                A = A1 / R1 + A2 / R2,
                B = B12 / R1 + B12 / R2,
                C = C1 / R1 + C2 / R2,
                a = -A/B,
                b = -C/B;

        mBisectorLine[0] = a;
        mBisectorLine[1] = b;

        mLanes[2] = mBisectorLine;

    }

    public int getVerticalLineCount() {
        return mVerticalLineCount;
    }

    public Mat getFrameWithLanes(int[] lanesColor) {
        createFrameWithLanes(lanesColor);
        return mFrameWithLanes;
    }

    public double[][] getLanes(){
        return mLanes;
    }

    public Mat getTemp(){
        return mTemp;
    }

}