package com.roee.opencv.opencv;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class LaneDetector {

    // max slope for a line to be considered as vertical
    private double VerticalThresholdSlope = 4;

    private Mat mRgba;
    private Mat mLines;
    private Mat mFrameWithLanes;
    private Mat mTemp;

    private ArrayList<LinearEquation> mLinearEquations;

    private LinearEquation mLeftLane;
    private LinearEquation mRightLane;
    private LinearEquation mBisectorLine;

    public LaneDetector(){
        mRgba = new Mat();
        mLines = new Mat();
        mLinearEquations = new ArrayList<LinearEquation>();
        mFrameWithLanes = new Mat();
        mTemp = new Mat();
    }

    public void processFrame(Mat frame){

        mLinearEquations.clear();

        mRgba = frame;

        // Detect all lines in image
        detectLines();
        // Transform the two dots received from HoughLinesP() into linear equations
        calculateLinearEquations();
        // Process lines to extract lanes
        extractLanes();
        // Find the angle bisector. Used for determination of tilt and deviation
        if(lanesFound()){
            calculateAngleBisector();
        }
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

        // Detect lines with Hough Transform
        Imgproc.HoughLinesP(mTemp, mLines, 1, Math.PI / 180, 35, 2, 100);

        // Release unnecessary temporary Mat
//        mTemp.release();

    }

    private void calculateLinearEquations() {

        Log.e("LaneDetector", "Detected lines: " + mLines.rows());

        for(int i=0;i<mLines.rows();i++){
            double[] vec = mLines.get(i, 0);
            double y1 = vec[0],
                    x1 = vec[1],
                    y2 = vec[2],
                    x2 = vec[3];

            // Find the linear equation parameters (y = a*x + b)
            double a = (y1 - y2) / (x1 - x2);
            double b = y1 - a * x1;

            mLinearEquations.add(new LinearEquation(a,b));
        }
    }

    public boolean qualifyAsLanes(LinearEquation line1, LinearEquation line2){
        double a1 = line1.getA(),
                b1 = line1.getB(),
                a2 = line2.getA(),
                b2 = line2.getB();

        if (Math.abs(b1 - b2) > 100 && Math.abs(a1) < VerticalThresholdSlope && Math.abs(a2) < VerticalThresholdSlope) {


            double  intersectionX = (b2 - b1)/(a1 - a2),
                    intersectionY = a1 * intersectionX + b1;

            Log.e("LaneDetector", "p(" + intersectionX + "," + intersectionY + ")");
            Log.e("LaneDetector", "Height: " + mRgba.height() + ", width:" + mRgba.width());

            // padding factor
            double pf = mRgba.height() * 0.5;

            if((intersectionX < 0 - pf || intersectionX > mRgba.height() + pf) || (intersectionY < 0 - pf || intersectionY > mRgba.width() + pf)) {
                return true;
            }
        }
        return false;
    }

    private void setLanes(LinearEquation lane1, LinearEquation lane2) {
        if(lane1.getA() > 0){
            mLeftLane = lane1;
            mRightLane = lane2;
        }else{
            mLeftLane = lane2;
            mRightLane = lane1;
        }
    }

    public void extractLanes() {

        Log.e("LaneDetector", "Equations: " + mLinearEquations.size());

        for(int i=0; i<mLinearEquations.size(); i++){
            for(int j=i+1; j<mLinearEquations.size(); j++){
                if(qualifyAsLanes(mLinearEquations.get(i), mLinearEquations.get(j))){
                    setLanes(mLinearEquations.get(i), mLinearEquations.get(j));
                    return;
                }
            }
        }

    }

    public void createFrameWithLanes(){
        mRgba.copyTo(mFrameWithLanes);
        if(lanesFound()){
            drawLinearEquation(mLeftLane, new Scalar(250, 0, 0));
            drawLinearEquation(mRightLane, new Scalar(0, 250, 0));
            drawLinearEquation(mBisectorLine, new Scalar(0, 0, 250));
        }
    }

    public void drawLinearEquation(LinearEquation line, Scalar color){
        double a = line.getA(),
                b = line.getB(),
                y1 = 0,
                y2 = mRgba.width(),
                x1 = a * y1 + b,
                x2 = a * y2 + b;

        if(a != 0 && b != 0){

            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            Imgproc.line(mFrameWithLanes,
                    start, end,
                    color,
                    4);
        }
    }

    public void calculateAngleBisector(){
        double a1 = mLeftLane.getA(),
                b1 = mLeftLane.getB(),
                a2 = mRightLane.getA(),
                b2 = mRightLane.getB(),
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

        mBisectorLine = new LinearEquation(a, b);
    }

    public boolean lanesFound(){
        return mLeftLane != null && mRightLane != null;
    }

    public Mat getFrameWithLanes() {
        createFrameWithLanes();
        return mFrameWithLanes;
    }

    public LinearEquation getBisectorLine(){
        return mBisectorLine;
    }

    public Mat getTemp(){
        return mTemp;
    }

}