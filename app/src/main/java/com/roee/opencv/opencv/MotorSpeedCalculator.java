package com.roee.opencv.opencv;

/**
 * Created by Roee on 10/10/2015.
 */
public class MotorSpeedCalculator {

    public static final int LEFT = 0;
    public static final int RIGHT = 1;
    public static final double LINE_HEIGHT = 1.0 / 2.0;
    private static final double K_TILT = 0.573;
    private static final double K_ERROR = 1.0 / 100.0;
    private static final double K_P = 1.6;
    private static final double K_D = 1.4;
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

    private double lastError;

    public MotorSpeedCalculator(double lastError) {
        this.lastError = lastError;
        mDistance = 0;
    }

    public static void setTiltCalibrationValue(double mTiltCalibrationValue) {
        MotorSpeedCalculator.sTiltCalibrationValue = mTiltCalibrationValue;
    }

    public static void setDeviationCalibrationValue(double mDeviationCalibrationValue) {
        MotorSpeedCalculator.sDeviationCalibrationValue = mDeviationCalibrationValue;
    }

    public static void resetSpeeds() {
        sLeftSpeed = BASE_SPEED;
        sRightSpeed = BASE_SPEED;
    }

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

    public double getTilt() {
        return mTilt / mFramesCount;
    }

    /*
    * The calibrated tilt is tan(theta), where theta is the angle between the direction of the car and the direction of the detected lane
    * Values should range from -1 to 1, where this function is the most accurate
     */
    public double getCalibratedTilt() {
        return (mTilt / mFramesCount + sTiltCalibrationValue + getCalibratedDeviation() / 5 * 0.02) * K_TILT;
    }

    public double getDeviation() {
        return mDeviation / mFramesCount;
    }

    public double getCalibratedDeviation() {
        return mDeviation / mFramesCount + sDeviationCalibrationValue;
    }

    public double getDistance() {
        return mDistance / mFramesCount;
    }

    public int getFramesCount() {
        return mFramesCount;
    }

    /**
     * Calculate the correction for each motor
     *
     * @param side
     * @return
     */
    private double correction(int side) {
        if (side == correctionDirection()) {
            return correctionMagnitude();
        }
        // Todo: check whether returning 0 instead is better or not
        return -correctionMagnitude();
    }

    public double getLeftSpeed() {
        return sLeftSpeed;
    }

    public double getRightSpeed() {
        return sRightSpeed;
    }

    public double error() {
        return K_ERROR * getDistance();
    }

    public double dError() {
        return error() - lastError;
    }

    public double correction() {
        return 1 / (1 + Math.exp(-4 * (K_P * error()))) - 0.5;
    }

    public int correctionDirection() {
        boolean tiltPositive = getCalibratedTilt() > 0;
        boolean deviationPositive = getCalibratedDeviation() > 0;

        if (tiltPositive && deviationPositive) {
            // Deviation: none, Tilt: right (RIGHT)
            return RIGHT;
        } else if (!tiltPositive && !deviationPositive) {
            // Deviation: none, Tilt: left (LEFT)
            return LEFT;
        } else if (!tiltPositive && deviationPositive) {
            // Deviation: right, Tilt: none (RIGHT)
            return RIGHT;
        } else if (tiltPositive && !deviationPositive) {
            // Deviation: left, Tilt: none (LEFT)
            return LEFT;
        }
        return -1;
    }

    public double correctionMagnitude() {
        double tilt = Math.abs(getCalibratedTilt());
        double deviation = Math.abs(getCalibratedDeviation());

        if (tilt > 3 || deviation > 20) {
            return Math.round((deviation / 250) * 100.0) / 100.0;
        }
        return 0;

    }

    public void calculateSpeeds() {
        sRightSpeed = BASE_SPEED + K_V * correction();
        sRightSpeed = (Math.abs(sRightSpeed) > MAX_SPEED) ? Math.signum(sRightSpeed) * MAX_SPEED : sRightSpeed;
        sLeftSpeed = BASE_SPEED + K_V * -correction();
        sLeftSpeed = (Math.abs(sLeftSpeed) > MAX_SPEED) ? Math.signum(sLeftSpeed) * MAX_SPEED : sLeftSpeed;
    }
}
