package com.roee.opencv.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;

/**
 * Created by Roee on 18/02/2016.
 */
public class ObstacleDetector {

    public static String TAG = ObstacleDetector.class.getSimpleName();

    private Mat mCanny;

    private boolean mObstacleDetected = false;

    /**
     * Detects obstacle in the frame
     * @param frame Canny frame
     */
    public void processFrame(Mat frame) {

        mCanny = frame;

        int whiteCount = Core.countNonZero(mCanny.submat((int) (DrivingActivity.mFrameHeight * (2.0 / 3.0)), (int) (DrivingActivity.mFrameHeight * (3.0 / 3.0) - 1),
                (int) (DrivingActivity.mFrameWidth * (1.0 / 3.0)), (int) (DrivingActivity.mFrameWidth * (2.0 / 3.0))));

        //Log.e(TAG, "processFrame: " + whiteCount);

        mObstacleDetected = whiteCount > 80 * ((DrivingActivity.mFrameHeight * DrivingActivity.mFrameWidth) / (352 * 288 / 2));

    }

    /**
     * @return whether an obstacle was detected or not
     */
    public boolean obstacleDetected() {
        return mObstacleDetected;
    }

}
