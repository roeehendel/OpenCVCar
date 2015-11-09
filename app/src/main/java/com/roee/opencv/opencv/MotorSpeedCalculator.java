package com.roee.opencv.opencv;

/**
 * Created by Roee on 10/10/2015.
 */
public class MotorSpeedCalculator {

    // Statuses
    public static final int STRAIGHT = 0;
    public static final int PRETURN_LEFT = 1;
    public static final int PRETURN_RIGHT = 2;
    public static final int TURN = 3;

    public static final int LEFT = 0;
    public static final int RIGHT = 1;

    public static final double CORRECTION_MAGNITUDE_THRESHOLD = 0.5;

    public static double mTiltCalibrationValue = 0;
    public static double mDeviationCalibrationValue = 0;

    private int mTotalVerticalLineCount = 0;
    private int mFramesCount = 0;

    private int mLeftOrRight = 0;
    private double mTilt = 0;
    private double mDeviation = 0;

    public int status(){
        double verticalLineAvgCount = (double)mTotalVerticalLineCount / (double)mFramesCount;
        if(verticalLineAvgCount > 1.6){
            // straight
            return STRAIGHT;
        }else if(verticalLineAvgCount > 0){
            // Determine the direction of the possible turn ahead
            if(mLeftOrRight > 0){
                return PRETURN_LEFT;
            }else{
                return PRETURN_RIGHT;
            }
        }
        return TURN;
    }

    public void addFrameData(double[][] lanes, int verticalLaneCount){

        mTotalVerticalLineCount += verticalLaneCount;
        mFramesCount++;

        for (int x = 0; x < lanes.length; x++) {
            // Get line y = a*x + b. [0] = a, [1] = b
            double[] line = lanes[x];

            double a = line[0],
                    b = line[1];

            mLeftOrRight += Math.signum(a);
        }

        mTilt += lanes[2][0] * 100;
        mDeviation += DrivingActivity.mFrameWidth/2 - lanes[2][1];

    }

    public double getTilt(){
        return mTilt / mFramesCount;
    }

    public double getCalibratedTilt(){
        return Math.round( (mTilt / mFramesCount + mTiltCalibrationValue) * 10.0 ) / 10.0;
    }

    public double getDeviation(){
        return mDeviation / mFramesCount;
    }

    public double getCalibratedDeviation(){
        return Math.round( (mDeviation / mFramesCount + mDeviationCalibrationValue) * 10.0 ) / 10.0;
    }

    public int getFramesCount(){
        return mFramesCount;
    }

    public int getTotalVerticalLineCount(){
        return mTotalVerticalLineCount;
    }

    public static void setTiltCalibrationValue(double mTiltCalibrationValue) {
        MotorSpeedCalculator.mTiltCalibrationValue = mTiltCalibrationValue;
    }

    public static void setDeviationCalibrationValue(double mDeviationCalibrationValue) {
        MotorSpeedCalculator.mDeviationCalibrationValue = mDeviationCalibrationValue;
    }

    /**
     * Calculate the correction for each motor
     *
     * @param side
     * @return
     */
    private double correction(int side){
        if(correctionMagnitude() > CORRECTION_MAGNITUDE_THRESHOLD){
            if(side == correctionDirection()){
                return 0.5;
            }
        }
        return 0;
    }

    public double getLeftSpeed(){
        return 1 + correction(LEFT);
    }

    public double getRightSpeed(){

        return 1.15 * (1 + correction(RIGHT));
    }

    public int correctionDirection(){
        boolean tiltPositive = getCalibratedTilt() > 0;
        boolean deviationPositive = getCalibratedDeviation() > 0;

        if(tiltPositive && deviationPositive){
            // Deviation: none, Tilt: right (RIGHT)
            return RIGHT;
        }else if(!tiltPositive && !deviationPositive){
            // Deviation: none, Tilt: left (LEFT)
            return LEFT;
        }else if(!tiltPositive && deviationPositive){
            // Deviation: right, Tilt: none (RIGHT)
            return RIGHT;
        }else if(tiltPositive && !deviationPositive){
            // Deviation: left, Tilt: none (LEFT)
            return LEFT;
        }
        return -1;
    }

    public double correctionMagnitude(){
        double tilt = getCalibratedTilt();
        double deviation = getCalibratedDeviation();

        if(Math.abs(tilt) > 3 && Math.abs(deviation) > 20){
            return 1;
        }
        return 0;

    }

}
