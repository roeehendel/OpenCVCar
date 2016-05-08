package com.roee.opencv.opencv;

/**
 * Created by Roee on 10/10/2015.
 */
public class MotorSpeedCalculator {


    public static final double LINE_HEIGHT = 1.0 / 2.0;
    private static final double K_TILT = 0.573;
    private static final double K_ERROR = 1.0 / 100.0;
    private static final double K_P = 1.6;
    private static final double K_V = 1.0;
    private static final double MAX_SPEED = 2;
    private static final double BASE_SPEED = 1;
    private static double sTiltCalibrationValue = 0;
    private static double sDeviationCalibrationValue = 0;
    private static double sLeftSpeed = BASE_SPEED;
    private static double sRightSpeed = BASE_SPEED;

    private int mFramesCount = 0;

    private double mTilt = 0;
    private double mDeviation = 0;
    private double mDistance = 0;

    public MotorSpeedCalculator() {
        mDistance = 0;
    }

    /**
     * Resets speeds to base speeds
     */
    public static void resetSpeeds() {
        sLeftSpeed = BASE_SPEED;
        sRightSpeed = BASE_SPEED;
    }

    /**
     * Adds a new frame data (bisectors)
     * @param bisectors the calculated bisectors
     */
    public void addFrameData(LinearEquation[] bisectors) {

        mFramesCount++;

        for (LinearEquation bisector : bisectors) {
            if (bisector != null) {
                mTilt += bisector.a / 2;
                mDeviation += (DrivingActivity.mFrameWidth / 2 - bisector.b) / 2;

                if (bisector.edgesCenter().x > DrivingActivity.mFrameHeight / 3) {
                    mDistance += bisector.distanceFromPoint(new Point(DrivingActivity.mFrameHeight * LINE_HEIGHT, DrivingActivity.mFrameWidth / 2)) / 4;
                    mDistance += bisector.distanceFromPoint(new Point(DrivingActivity.mFrameHeight * LINE_HEIGHT * 0.5, DrivingActivity.mFrameWidth / 2)) / 4;
                }

            }
        }

    }

    /**
     * Gets the average tilt
     * @return average tilt
     */
    public double getTilt() {
        return mTilt / mFramesCount;
    }

    /**
     * Gets the calibrated average tilt
     * @return calibrated average tilt
     */
    public double getCalibratedTilt() {
        return (mTilt / mFramesCount + sTiltCalibrationValue + getCalibratedDeviation() / 5 * 0.02) * K_TILT;
    }

    /**
     * Gets the average deviation
     * @return average deviation
     */
    public double getDeviation() {
        return mDeviation / mFramesCount;
    }

    /**
     * Gets the calibrated average deviation
     * @return calibrated average deviation
     */
    public double getCalibratedDeviation() {
        return mDeviation / mFramesCount + sDeviationCalibrationValue;
    }

    /**
     * Gets the average distance
     * @return average distance
     */
    public double getDistance() {
        return mDistance / mFramesCount;
    }

    /**
     * Gets the frame count
     * @return frame count
     */
    public int getFramesCount() {
        return mFramesCount;
    }

    /**
     * Gets the speed of the left motor
     * @return left motor speed
     */
    public double getLeftSpeed() {
        return sLeftSpeed;
    }

    /**
     * Gets the speed of the right motor
     * @return right motor speed
     */
    public double getRightSpeed() {
        return sRightSpeed;
    }

    /**
     * Gets the speed of the right motor
     * @return right motor speed
     */
    public double error() {
        return K_ERROR * getDistance();
    }

    /**
     * Correction function
     * @return correction
     */
    public double correction() {
        return 1 / (1 + Math.exp(-4 * (K_P * error()))) - 0.5;
    }

    /**
     * Calculates speeds for each motor based on the correction
     */
    public void calculateSpeeds() {
        sRightSpeed = BASE_SPEED + K_V * correction();
        sRightSpeed = (Math.abs(sRightSpeed) > MAX_SPEED) ? Math.signum(sRightSpeed) * MAX_SPEED : sRightSpeed;
        sLeftSpeed = BASE_SPEED + K_V * -correction();
        sLeftSpeed = (Math.abs(sLeftSpeed) > MAX_SPEED) ? Math.signum(sLeftSpeed) * MAX_SPEED : sLeftSpeed;
    }
}
