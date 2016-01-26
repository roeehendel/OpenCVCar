package com.roee.opencv.opencv;

/**
 * Created by Roee on 10/10/2015.
 */
public class MotorSpeedCalculator {

    public static final int LEFT = 0;
    public static final int RIGHT = 1;

    private static final double TILT_CORRECTION = 0.573;

    private static final int V0 = 1;

    public static double mTiltCalibrationValue = 0;
    public static double mDeviationCalibrationValue = 0;

    private int mFramesCount = 0;

    private double mTilt = 0;
    private double mDeviation = 0;


    public void addFrameData(LinearEquation bisector){

        mFramesCount++;

        mTilt += bisector.getA();
        mDeviation += DrivingActivity.mFrameWidth/2 - bisector.getB();

    }

    public double getTilt(){
        return mTilt / mFramesCount;
    }

    /*
    * The calibrated tilt is tan(theta), where theta is the angle between the direction of the car and the direction of the detected lane
    * Values should range from -1 to 1, where this function is the most accurate
     */
    public double getCalibratedTilt(){
        return Math.round( (mTilt / mFramesCount + mTiltCalibrationValue + getCalibratedDeviation()/5*0.02) * TILT_CORRECTION * 100.0 ) / 100.0;
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
        if(side == correctionDirection()) {
            return correctionMagnitude();
        }
        // Todo: check whether returning 0 instead is better or not
        return -correctionMagnitude();
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
        double tilt = Math.abs(getCalibratedTilt());
        double deviation = Math.abs(getCalibratedDeviation());

        if(tilt > 3 || deviation > 20){
            return Math.round((deviation / 250) * 100.0) / 100.0;
        }
        return 0;

    }

}
