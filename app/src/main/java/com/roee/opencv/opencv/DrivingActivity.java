package com.roee.opencv.opencv;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.PowerManager;
import android.util.Log;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class DrivingActivity extends Activity implements CvCameraViewListener2 {
    public static final int BASE_WIDTH = 352 / 2;
    public static final int BASE_HEIGHT = 288 / 2;
    public static final double SCALE = 1;
    public static final boolean USE_CAMERA_NATIVE_SIZE = false;
    public static final int MESSAGE_TOAST = 1;
    public static final int MESSAGE_STATE_CHANGE = 2;
    public static final String TOAST = "toast";
    private static final String TAG = DrivingActivity.class.getSimpleName();
    private static final String PREFS_NAME = "PrefsFile";
    private static final String POWER_PREF = "power";
    private static final int REQUEST_ENABLE_BT = 1;
    private static final int REQUEST_CONNECT_DEVICE = 2;
    private static final int REQUEST_SETTINGS = 3;
    public static int mFrameWidth = (int) (BASE_WIDTH * SCALE);
    public static int mFrameHeight = (int) (BASE_HEIGHT * SCALE);
    private Mat mRgba;
    private Mat mDisplayFrame;
    private Mat mTemp;
    private int mFrameCount = 0;
    private int mFramesPerCalculation = 1;
    private int mNotDetectedCount;

    private CameraBridgeViewBase mOpenCvCameraView;
    private TextView mVerticalLinesTextView;
    private TextView mStatusTextView;
    private TextView mAsymmetryTextView;
    private TextView mPowerTextView;
    private Button mStartDriveButton;
    private Spinner mPreviewImageSpinner;

    private enum PreviewImageType {
        GRAYSCALE,
        CANNY,
        HOUGH,
        FINAL
    }

    private PreviewImageType mPreviewImageType;

    /*
    * Bluetooth NXT connection
     */
    private Button mStopDriveButton;
    private SeekBar mSpeedSeekBar;
    private LaneDetector mLaneDetector;
    private ObstacleDetector mObstacleDetector;
    private MotorSpeedCalculator mMotorSpeedCalculator;
    private SharedPreferences mSharedPreferences;
    private boolean NO_BT = false;
    private BluetoothAdapter mBluetoothAdapter;
    private PowerManager mPowerManager;
    private PowerManager.WakeLock mWakeLock;
    private NXTTalker mNXTTalker;

    private int mState = NXTTalker.STATE_NONE;
    private int mSavedState = NXTTalker.STATE_NONE;
    private boolean mNewLaunch = true;
    private String mDeviceAddress = null;
    private TextView mStateDisplay;
    private Button mConnectButton;
    private Button mDisconnectButton;

    private int mPower;
    private boolean mDrive = false;
    private boolean mDriving = false;

    private byte leftSpeed = 0;
    private byte rightSpeed = 0;

    private boolean mReverse;
    private boolean mReverseLR;
    private boolean mRegulateSpeed;
    private boolean mSynchronizeMotors;

    private Handler mHandler = new Handler() {

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case MESSAGE_TOAST:
                    Toast.makeText(getApplicationContext(),
                            msg.getData().getString(TOAST), Toast.LENGTH_SHORT).show();
                    break;
                case MESSAGE_STATE_CHANGE:
                    mState = msg.arg1;
                    displayState();
                    break;
            }
        }
    };
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    /**
     * Displays Bluetooth connection state
     */
    private void displayState() {
        String stateText = null;
        int color = 0;
        switch (mState) {
            case NXTTalker.STATE_NONE:
                stateText = "Not connected";
                color = 0xffff0000;
                mConnectButton.setVisibility(View.VISIBLE);
                mDisconnectButton.setVisibility(View.GONE);
                setProgressBarIndeterminateVisibility(false);
                if (mWakeLock.isHeld()) {
                    mWakeLock.release();
                }
                break;
            case NXTTalker.STATE_CONNECTING:
                stateText = "Connecting...";
                color = 0xffffff00;
                mConnectButton.setVisibility(View.GONE);
                mDisconnectButton.setVisibility(View.GONE);
                setProgressBarIndeterminateVisibility(true);
                if (!mWakeLock.isHeld()) {
                    mWakeLock.acquire();
                }
                break;
            case NXTTalker.STATE_CONNECTED:
                stateText = "Connected";
                color = 0xff00ff00;
                mConnectButton.setVisibility(View.GONE);
                mDisconnectButton.setVisibility(View.VISIBLE);
                setProgressBarIndeterminateVisibility(false);
                if (!mWakeLock.isHeld()) {
                    mWakeLock.acquire();
                }
                break;
        }
        mStateDisplay.setText(stateText);
        mStateDisplay.setTextColor(color);
    }

    /**
     * Called when the activity is created
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.detection_preview_view);

        mSharedPreferences = getSharedPreferences(PREFS_NAME, 0);
        mPower = mSharedPreferences.getInt(POWER_PREF, 35);

        mVerticalLinesTextView = (TextView) findViewById(R.id.vertical_lanes_text_view);
        mStatusTextView = (TextView) findViewById(R.id.status_text_view);
        mAsymmetryTextView = (TextView) findViewById(R.id.asymmetry_text_view);
        mPowerTextView = (TextView) findViewById(R.id.power_text_view);

        mPowerTextView.setText(Integer.toString(mPower));

        mStartDriveButton = (Button) findViewById(R.id.start_drive_button);
        mStartDriveButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mDrive = true;
                mStartDriveButton.setVisibility(View.GONE);
                mStopDriveButton.setVisibility(View.VISIBLE);
            }
        });

        mStopDriveButton = (Button) findViewById(R.id.stop_drive_button);
        mStopDriveButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mDrive = false;
                mNXTTalker.motors((byte) 0, (byte) 0, mRegulateSpeed, mSynchronizeMotors);
                MotorSpeedCalculator.resetSpeeds();
                mStartDriveButton.setVisibility(View.VISIBLE);
                mStopDriveButton.setVisibility(View.GONE);
            }
        });

        mSpeedSeekBar = (SeekBar) findViewById(R.id.power_seek_bar);
        mSpeedSeekBar.setProgress(mPower);
        mSpeedSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                mPower = 5 * (Math.round(i / 5));
                mPowerTextView.setText(Integer.toString(mPower));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        mPreviewImageSpinner = (Spinner) findViewById(R.id.preview_image_spinner);

        // Create an ArrayAdapter using the string array and a default spinner layout
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,
                R.array.preview_image_array, android.R.layout.simple_spinner_item);
        // Specify the layout to use when the list of choices appears
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        // Apply the adapter to the spinner
        mPreviewImageSpinner.setAdapter(adapter);

        mPreviewImageSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> adapterView, View view, int pos, long id) {
                mPreviewImageType = PreviewImageType.values()[pos];
            }

            @Override
            public void onNothingSelected(AdapterView<?> adapterView) {

            }
        });

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(
                R.id.lane_detection_activity_surface_view);

        // Limit frame size for speed.
        if (!USE_CAMERA_NATIVE_SIZE) {
            mOpenCvCameraView.setMaxFrameSize(mFrameWidth, mFrameHeight);
        }

        mOpenCvCameraView.setCvCameraViewListener(this);

        /*
        * Bluetooth NXT connection
         */

        mPowerManager = (PowerManager) getSystemService(Context.POWER_SERVICE);
        mWakeLock = mPowerManager.newWakeLock(PowerManager.SCREEN_DIM_WAKE_LOCK |
                PowerManager.ON_AFTER_RELEASE, "NXT Remote Control");

        if (!NO_BT) {
            mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

            if (mBluetoothAdapter == null) {
                Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
                finish();
                return;
            }
        }

        mNXTTalker = new NXTTalker(mHandler);

        mStateDisplay = (TextView) findViewById(R.id.state_text_view);

        mConnectButton = (Button) findViewById(R.id.connect_button);
        mConnectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (!NO_BT) {
                    findBrick();
                } else {
                    mState = NXTTalker.STATE_CONNECTED;
                    displayState();
                }
            }
        });

        mDisconnectButton = (Button) findViewById(R.id.disconnect_button);
        mDisconnectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mNXTTalker.stop();
            }
        });

    }

    /**
     * Called when the activity is started
     */
    @Override
    protected void onStart() {
        super.onStart();
        if (!NO_BT) {
            if (!mBluetoothAdapter.isEnabled()) {
                Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
            } else {
                if (mSavedState == NXTTalker.STATE_CONNECTED) {
                    BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(mDeviceAddress);
                    mNXTTalker.connect(device);
                }
            }
        }
    }

    /**
     * Starts bluetooth device selection activity (ChooseDeviceActivity)
     */
    private void findBrick() {
        Intent intent = new Intent(this, ChooseDeviceActivity.class);
        startActivityForResult(intent, REQUEST_CONNECT_DEVICE);
    }

    /**
     * Called when ChooseDeviceActivity exists with a result
     */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        switch (requestCode) {
            case REQUEST_ENABLE_BT:
                if (resultCode == Activity.RESULT_OK) {
                    findBrick();
                } else {
                    Toast.makeText(this, "Bluetooth not enabled, exiting.",
                            Toast.LENGTH_LONG).show();
                    finish();
                }
                break;
            case REQUEST_CONNECT_DEVICE:
                if (resultCode == Activity.RESULT_OK) {
                    String address = data.getExtras()
                            .getString(ChooseDeviceActivity.EXTRA_DEVICE_ADDRESS);
                    BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
                    //Toast.makeText(this, address, Toast.LENGTH_LONG).show();
                    mDeviceAddress = address;
                    mNXTTalker.connect(device);
                }
                break;
            case REQUEST_SETTINGS:
                //XXX?
                break;
        }
    }

    /**
     * Called when the activity is stopped
     */
    @Override
    protected void onStop() {
        super.onStop();
        //Log.i("NXT", "NXTRemoteControl.onStop()");
        mSavedState = mState;
        mNXTTalker.stop();
        if (mWakeLock.isHeld()) {
            mWakeLock.release();
        }
    }


    /**
     * Formats a double number for display
     * It rounds the number and leaves two decimal placed after the dot
     *
     * @param num original number
     * @return number formatted for display
     */
    private double formatForDisplay(double num) {
        return Math.round(num * 100.0) / 100.0;
    }

    /**
     * Updates the top bar with data from the MotorSpeedCalculator
     *
     * @param motorSpeedCalculator the MotorSpeedCalculator from which to get data to display
     */
    public void updateTopBar(final MotorSpeedCalculator motorSpeedCalculator) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (motorSpeedCalculator != null && motorSpeedCalculator.getFramesCount() != 0) {
                    mAsymmetryTextView.setText(
                            String.format("Dist= %s, Drive= %s",
                                    formatForDisplay(motorSpeedCalculator.getDistance()),
                                    mDriving
                            ));
                    mVerticalLinesTextView.setText(
                            String.format("L= %s, R= %s",
                                    formatForDisplay(motorSpeedCalculator.getLeftSpeed()),
                                    formatForDisplay(motorSpeedCalculator.getRightSpeed())
                            ));
                    mStatusTextView.setText(
                            String.format("Cor= %s",
                                    formatForDisplay(motorSpeedCalculator.correction())
                            ));
                }
            }
        });
    }

    /**
     * Called when the activity is paused
     */
    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        SharedPreferences.Editor editor = mSharedPreferences.edit();
        editor.putInt(POWER_PREF, mPower);
        editor.apply();
    }

    /**
     * Called when the activity is resumed
     */
    @Override
    public void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG,
                    "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    /**
     * Called when the activity is destroyed
     */
    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    /**
     * Called once when the camera view is started
     * Initiates image matrices and other objects and values
     *
     * @param width  -  the width of the frames that will be delivered
     * @param height - the height of the frames that will be delivered
     */
    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mTemp = new Mat(height, width, CvType.CV_8UC1);
        mLaneDetector = new LaneDetector();
        mObstacleDetector = new ObstacleDetector();
        mMotorSpeedCalculator = new MotorSpeedCalculator();
        mFrameHeight = height;
        mFrameWidth = width;
    }

    /**
     * Called once when the camera view is stopped
     * Releases the memory used by the matrices
     */
    @Override
    public void onCameraViewStopped() {
        mRgba.release();
        mTemp.release();
    }


    /**
     * Called each time a frame is received from the camera.
     * Uses detectors to process the frame and determine the speeds of the motors.
     * Finally, sends the speeds to the NXT using NXTTalker.
     *
     * @param inputFrame raw camera frame
     * @return frame to display as preview
     */
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();

        // Process visual data to find lanes
        mLaneDetector.processFrame(mRgba);
        // Detect obstacles
        mObstacleDetector.processFrame(mLaneDetector.getCanny());

        //Log.e(TAG, "onCameraFrame: " + mObstacleDetector.obstacleDetected());

        mDriving = mDrive && !mObstacleDetector.obstacleDetected();

        if (!mDriving) {
            leftSpeed = (byte) (0);
            rightSpeed = (byte) (0);
        }

        LinearEquation[] bisectors = mLaneDetector.getBisectorLines();

        if (bisectors[0] != null) {
            mMotorSpeedCalculator.addFrameData(bisectors);
            mFrameCount++;
        } else {
            mNotDetectedCount++;
            if (mNotDetectedCount > 5) {
                mNotDetectedCount = 0;
                leftSpeed = (byte) (0);
                rightSpeed = (byte) (0);
            }
        }

        if (mFrameCount == mFramesPerCalculation) {

            // Reset frame counter
            mFrameCount = 0;

            // Calculate speeds
            mMotorSpeedCalculator.calculateSpeeds();


            if (mDriving) {
                leftSpeed = (byte) (mPower * mMotorSpeedCalculator.getLeftSpeed());
                rightSpeed = (byte) (mPower * mMotorSpeedCalculator.getRightSpeed());
            } else {
                leftSpeed = (byte) (0);
                rightSpeed = (byte) (0);
            }

            // Update the top bar with the frame's MotorSpeedCalculator
            updateTopBar(mMotorSpeedCalculator);

            mMotorSpeedCalculator = new MotorSpeedCalculator();

        }

        // Sends speeds to the NXT
        mNXTTalker.motors(leftSpeed, rightSpeed, mRegulateSpeed, mSynchronizeMotors);

        // Generate preview frame
        switch (mPreviewImageType){
            case GRAYSCALE:
                mDisplayFrame = mLaneDetector.getGrayscale();
                break;
            case CANNY:
                mDisplayFrame = mLaneDetector.getCanny();
                break;
            case HOUGH:
                mDisplayFrame = mLaneDetector.getHough();
                break;
            default:
                mDisplayFrame = mLaneDetector.getDisplayFrame();
                break;
        }


//        return mLaneDetector.getTemp();
        return mDisplayFrame;
    }

}