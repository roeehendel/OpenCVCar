package com.roee.opencv.opencv;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class LaneDetector {

    private static final String LOG_TAG = LaneDetector.class.getSimpleName();

    private static final int LEFT = 0;
    private static final int RIGHT = 1;

    // max slope for a line to be considered as vertical
    private static final double VERTICAL_THRESHOLD_SLOPE = 6;
    private static final double BRIGHTNESS_DIFFERENCE_THRESHOLD = 15;
    private static final int LENGTH_THRESHOLD = DrivingActivity.mFrameWidth / 10;

    private Mat mRgba;
    private Mat mGrayscale;
    private Mat mDisplayFrame;
    private Mat mLines;
    private Mat mCanny;

    private ArrayList<LinearEquation> mLinearEquations;

    private static LinearEquation[][] mLanes = new LinearEquation[2][2];
    private static LinearEquation[] mBisectorLines = new LinearEquation[2];


    public LaneDetector(){
        mRgba = new Mat();
        mLines = new Mat();
        mLinearEquations = new ArrayList<LinearEquation>();
        mGrayscale = new Mat();
        mDisplayFrame = new Mat();
        mCanny = new Mat();
    }

    public void processFrame(Mat frame){

        mLinearEquations.clear();
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                mLanes[i][j] = null;
            }
        }
        mRgba = frame;

        // Detect all lines in image
        detectLines();
        // Transform the two dots received from HoughLinesP() into linear equations
        long start = System.nanoTime();
        calculateLinearEquations();
        double end = System.nanoTime();
        //Log.e(LOG_TAG, "calculateLinearEquations: " + Double.toString((end - start) / 1000000000));
        // Process lines to extract lanes
        start = System.nanoTime();
        extractLanes();
        end = System.nanoTime();
        //Log.e(LOG_TAG, "extractLanes: " + Double.toString((end - start) / 1000000000));
        // Find the angle bisector. Used for determination of tilt and deviation
        for(int i=0;i<2;i++){
            if(lanesFound(i)){
                mBisectorLines[i] = LinearEquation.calculateAngleBisector(mLanes[LEFT][i], mLanes[RIGHT][i]);
            }
        }
    }

    public void detectLines(){

        // Convert image to grayscale
        Imgproc.cvtColor(mRgba, mGrayscale, Imgproc.COLOR_RGB2GRAY);

        double otsu_thresh_val = Imgproc.threshold(
                mGrayscale, new Mat(), 0, 255, Imgproc.THRESH_OTSU | Imgproc.THRESH_BINARY);

        // Apply Canny to image
        double threshold1 = otsu_thresh_val / 2;
        double threshold2 = otsu_thresh_val;
        Imgproc.Canny(mGrayscale, mCanny, threshold1, threshold2);

        long start = System.nanoTime();
        // Detect lines with Hough Transform
        Imgproc.HoughLinesP(mCanny, mLines, 1, Math.PI / 180, 30, LENGTH_THRESHOLD, 20);
        double end = System.nanoTime();

        //Log.e(LOG_TAG, "detectLines: " + Double.toString((end-start) / 1000000000));
        
        // Release unnecessary temporary Mat
//        mCanny.release();

    }

    private void calculateLinearEquations() {

        //Log.e("LaneDetector", "Detected lines: " + mLines.rows());

        for(int i=0;i<mLines.rows();i++){
            double[] vec = mLines.get(i, 0);
            double y1 = vec[0],
                    x1 = vec[1],
                    y2 = vec[2],
                    x2 = vec[3];
            LinearEquation linearEquation = new LinearEquation(x1, y1, x2, y2);
            if(linearEquation.length() > LENGTH_THRESHOLD){
                mLinearEquations.add(linearEquation);
            }
        }
    }



    public boolean qualifyAsLanes(LinearEquation line1, LinearEquation line2){
        double a1 = line1.a,
                b1 = line1.b,
                a2 = line2.a,
                b2 = line2.b;

//        for(int i=0;i<2;i++){
//            for(LinearEquation lane: mLanes[i]){
//                if(lane != null){
//                    //Log.e(LOG_TAG, "qualifyAsLanes: " + Math.abs(lane.a - a1) + ", " + Math.abs(lane.a - a2));
//                    if(Math.abs(lane.a - a1) < 0.3 || Math.abs(lane.a - a2) < 0.3){
//                        return false;
//                    }
//                }
//            }
//        }

        if (Math.abs(b1 - b2) > 10 && Math.abs(a1) < VERTICAL_THRESHOLD_SLOPE && Math.abs(a2) < VERTICAL_THRESHOLD_SLOPE) {

            Point intersection = LinearEquation.intersect(line1, line2);

            //Log.e("LaneDetector", "Height: " + mRgba.height() + ", width:" + mRgba.width());

            // padding factor
            double pf = mRgba.height() * 0.5;

            if((intersection.x < 0 - pf || intersection.x > mRgba.height() + pf) || (intersection.y < 0 - pf || intersection.y > mRgba.width() + pf)) {
                if(intersection.x < 0 && Math.abs(intersection.x) < 400) {
                    if(brightnessDifferenceFits(line1) && brightnessDifferenceFits(line2)){
                        return true;
                    }
                }
            }
        }
        return false;
    }

    private void setLanes(LinearEquation lane1, LinearEquation lane2, int index) {
        if(lane1.distanceFromPoint(new Point(DrivingActivity.mFrameHeight/2, 0)) > lane2.distanceFromPoint(new Point(DrivingActivity.mFrameHeight/2, 0))){
            mLanes[LEFT][index] = lane1;
            mLanes[RIGHT][index] = lane2;
        }else{
            mLanes[LEFT][index] = lane2;
            mLanes[RIGHT][index] = lane1;
        }
    }

    public void extractLanes() {

        //Log.e("LaneDetector", "Equations: " + mLinearEquations.size());

        int index = 0;

        for(int i=0; i<mLinearEquations.size(); i++){
            for(int j=i+1; j<mLinearEquations.size(); j++){
                if(qualifyAsLanes(mLinearEquations.get(i), mLinearEquations.get(j))){
                    setLanes(mLinearEquations.get(i), mLinearEquations.get(j), index);
                    Log.e(LOG_TAG, "extractLanes: " + index);
                    index++;
                    if(index>1){
                        return;
                    }
                }
            }
        }

    }

    public boolean brightnessDifferenceFits(LinearEquation line){
        return Math.abs(brightnessDifferenceAroundLine(line)) > BRIGHTNESS_DIFFERENCE_THRESHOLD;
    }

//    public void filterBybrightnessDifference(){
//        ArrayList<LinearEquation> iterable = (ArrayList<LinearEquation>) mLinearEquations.clone();
//        for(LinearEquation line: iterable){
//            if(Math.abs(brightnessDifferenceAroundLine(line)) < BRIGHTNESS_DIFFERENCE_THRESHOLD){
//                mLinearEquations.remove(line);
//            }
//        }
//    }

    public double brightnessDifferenceAroundLine(LinearEquation line){
        LinearEquation side1 = new LinearEquation(line.a, line.b + 10);
        LinearEquation side2 = new LinearEquation(line.a, line.b - 10);

        return lineBrightness(side1) - lineBrightness(side2);
    }

    public double lineBrightness(LinearEquation line){
        Point p;
        double sum = 0;
        int samples = 0;
        for(double i = line.point1.x;i<line.point2.x; i++){
            p = new Point(i, line.y(i));
            double[] point = mGrayscale.get((int)Math.round(p.x), (int)Math.round(p.y));
            if(point != null){
                sum += point[0];
                samples++;
            }
        }

        return sum / samples;
    }

    public void drawLanes(Mat frame){

            for(int i=0;i<2;i++) {
                if (lanesFound(i)) {
                    int color = 250;
                    drawLinearEquation(frame, mLanes[LEFT][i], new Scalar(color, 0, 0));
                    drawLinearEquation(frame, mLanes[RIGHT][i], new Scalar(0, color, 0));
                    drawLinearEquation(frame, mBisectorLines[i], new Scalar(0, 0, color));

//                    Log.e(LOG_TAG, "Left=" + brightnessDifferenceAroundLine(mLanes[LEFT][0]));
//                    Log.e(LOG_TAG, "Right=" + brightnessDifferenceAroundLine(mLanes[RIGHT][0]));
//                    Log.e(LOG_TAG, "Center=" + brightnessDifferenceAroundLine(mBisectorLines[0]));
                }
            }
    }

    public void drawLines(Mat frame){
        for(LinearEquation line: mLinearEquations){
            drawLinearEquation(frame, line, new Scalar(0, 0, 0));
        }
    }

    public void createDisplayFrame(){
        //mCanny.copyTo(mDisplayFrame);
        mRgba.copyTo(mDisplayFrame);
        //drawLines(mDisplayFrame);
        drawLanes(mDisplayFrame);
    }

    public void drawLinearEquation(Mat frame, LinearEquation line, Scalar color){
        double a = line.a,
                b = line.b,
                y1 = 0,
                y2 = mRgba.width(),
                x1 = a * y1 + b,
                x2 = a * y2 + b;

        if(a != 0 && b != 0){

            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            Imgproc.line(frame,
                    start, end,
                    color,
                    4);
        }
    }

    public boolean lanesFound(int index){
        return mLanes[LEFT][index] != null && mLanes[RIGHT][index] != null;
    }

    public Mat getDisplayFrame(){
        createDisplayFrame();
        return mDisplayFrame;
    }

    public LinearEquation getBisectorLine(int index){
        return mBisectorLines[index];
    }

    public Mat getCanny(){
        return mCanny;
    }

    public LinearEquation[] getBisectorLines() {
        return mBisectorLines;
    }
}