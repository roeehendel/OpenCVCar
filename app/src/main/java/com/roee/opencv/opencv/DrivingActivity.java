package com.roee.opencv.opencv;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.PowerManager;
import android.util.Log;
import android.view.Menu;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
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

import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class DrivingActivity extends Activity implements CvCameraViewListener2 {
    private static final String TAG = DrivingActivity.class.getSimpleName();

    private int mAccentColorInt;
    private int[] mAccentColor;

    public static int mFrameWidth = 320;
    public static int mFrameHeight = 240;

    public final String[] statusCodes = {"straight", "preturn left", "preturn right", "turn"};

    private Mat mRgba;
    private Mat mProcessedFrame;
    private Mat mTemp;

    private int mFrameCount = 0;
    private int mFramesPerCalculation = 4;
    private int mStatus;
    private boolean mCalibrateOnNextCalc = false;

    private CameraBridgeViewBase mOpenCvCameraView;

    private TextView mVerticalLinesTextView;
    private TextView mStatusTextView;
    private TextView mAsymmetryTextView;

    private Button mCalibrateButton;
    private Button mStartDriveButton;
    private Button mStopDriveButton;

    private LaneDetector mLaneDetector;

    private MotorSpeedCalculator mMotorSpeedCalculator;

    public static int NUMBER_OF_CORES = Runtime.getRuntime().availableProcessors();
    ThreadPoolExecutor executor = new ThreadPoolExecutor(
            NUMBER_OF_CORES,
            NUMBER_OF_CORES,
            60L,
            TimeUnit.SECONDS,
            new LinkedBlockingQueue<Runnable>()
    );

    /*
    * Bluetooth NXT connection
     */

    private boolean NO_BT = false;

    private static final int REQUEST_ENABLE_BT = 1;
    private static final int REQUEST_CONNECT_DEVICE = 2;
    private static final int REQUEST_SETTINGS = 3;

    public static final int MESSAGE_TOAST = 1;
    public static final int MESSAGE_STATE_CHANGE = 2;

    public static final String TOAST = "toast";

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
    private Menu mMenu;

    private int mPower = 40;
    private boolean mDrive = false;

    private boolean mReverse;
    private boolean mReverseLR;
    private boolean mRegulateSpeed;
    private boolean mSynchronizeMotors;

    private Handler mHandler = new Handler() {

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case MESSAGE_TOAST:
                    Toast.makeText(getApplicationContext(), msg.getData().getString(TOAST), Toast.LENGTH_SHORT).show();
                    break;
                case MESSAGE_STATE_CHANGE:
                    mState = msg.arg1;
                    displayState();
                    break;
            }
        }
    };

    private void displayState() {
        String stateText = null;
        int color = 0;
        switch (mState){
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

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                }
                break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public DrivingActivity() {
        Log.i(TAG, "Instantiated new " + TAG);
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.detection_preview_view);

        mAccentColorInt = getResources().getColor(R.color.accent_material_light_1);
        mAccentColor = new int[]{Color.red(mAccentColorInt), Color.green(mAccentColorInt), Color.blue(mAccentColorInt)};

        mVerticalLinesTextView = (TextView) findViewById(R.id.vertical_lanes_text_view);
        mStatusTextView = (TextView) findViewById(R.id.status_text_view);
        mAsymmetryTextView = (TextView) findViewById(R.id.asymmetry_text_view);

        mCalibrateButton = (Button) findViewById(R.id.calibrate_button);
        mCalibrateButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                calibrateAsymmetry();
            }
        });

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
                mStartDriveButton.setVisibility(View.VISIBLE);
                mStopDriveButton.setVisibility(View.GONE);
            }
        });

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.lane_detection_activity_surface_view);

        // Limit frame size for speed.
        mOpenCvCameraView.setMaxFrameSize(mFrameWidth,mFrameHeight);

        mOpenCvCameraView.setCvCameraViewListener(this);

        /*
        * Bluetooth NXT connection
         */

        mPowerManager = (PowerManager) getSystemService(Context.POWER_SERVICE);
        mWakeLock = mPowerManager.newWakeLock(PowerManager.SCREEN_DIM_WAKE_LOCK | PowerManager.ON_AFTER_RELEASE, "NXT Remote Control");

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

    /*
    * Starts bluetooth device selection activity
     */
    private void findBrick() {
        Intent intent = new Intent(this, ChooseDeviceActivity.class);
        startActivityForResult(intent, REQUEST_CONNECT_DEVICE);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        switch (requestCode) {
            case REQUEST_ENABLE_BT:
                if (resultCode == Activity.RESULT_OK) {
                    findBrick();
                } else {
                    Toast.makeText(this, "Bluetooth not enabled, exiting.", Toast.LENGTH_LONG).show();
                    finish();
                }
                break;
            case REQUEST_CONNECT_DEVICE:
                if (resultCode == Activity.RESULT_OK) {
                    String address = data.getExtras().getString(ChooseDeviceActivity.EXTRA_DEVICE_ADDRESS);
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

    private void calibrateAsymmetry() {
        mCalibrateOnNextCalc = true;
    }

    private Handler mTopBarHandler = new Handler();

    private Runnable mUpdateTopBarTask = new Runnable() {
        public void run() {
            if (mMotorSpeedCalculator != null && mMotorSpeedCalculator.getFramesCount() != 0) {
                mAsymmetryTextView.setText("Tilt: " + mMotorSpeedCalculator.getCalibratedTilt() + ", Deviation: " + mMotorSpeedCalculator.getCalibratedDeviation());
                String displayDirection = (mMotorSpeedCalculator.correctionDirection() == MotorSpeedCalculator.LEFT) ? "Left" : "Right";
                mVerticalLinesTextView.setText("Dir: " + displayDirection + ", Mag: " + mMotorSpeedCalculator.correctionMagnitude());
                mStatusTextView.setText("Stts: " + statusCodes[mStatus]);
            }
            mTopBarHandler.postDelayed(mUpdateTopBarTask, 100);
        }
    };

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        mTopBarHandler.removeCallbacks(mUpdateTopBarTask);

    }

    @Override
    public void onResume()
    {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        mTopBarHandler.removeCallbacks(mUpdateTopBarTask);
        mTopBarHandler.postDelayed(mUpdateTopBarTask, 100);
    }


    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mTemp = new Mat(height, width, CvType.CV_8UC1);
        mLaneDetector = new LaneDetector();
        mMotorSpeedCalculator = new MotorSpeedCalculator();
    }

    public void onCameraViewStopped() {
        mRgba.release();
        mTemp.release();
    }



    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();

        // Process visual data to find lanes
        mLaneDetector.proccessFrame(mRgba);

        mFrameCount++;


        mMotorSpeedCalculator.addFrameData(mLaneDetector.getLanes(), mLaneDetector.getVerticalLineCount());

        if (mFrameCount == mFramesPerCalculation) {

            // Reset frame counter
            mFrameCount = 0;
            // Calculate motor speeds
            mStatus = mMotorSpeedCalculator.status();

            if (mCalibrateOnNextCalc) {
                mCalibrateOnNextCalc = false;
                MotorSpeedCalculator.setTiltCalibrationValue(-mMotorSpeedCalculator.getTilt());
                MotorSpeedCalculator.setDeviationCalibrationValue(-mMotorSpeedCalculator.getDeviation());
            }

            byte l;
            byte r;

            if (mDrive && mMotorSpeedCalculator.getTotalVerticalLineCount() / mMotorSpeedCalculator.getFramesCount() == 2) {
                l = (byte) (mPower * mMotorSpeedCalculator.getLeftSpeed());
                r = (byte) (mPower * mMotorSpeedCalculator.getRightSpeed());
            } else {
                l = (byte) (0);
                r = (byte) (0);
            }

            mNXTTalker.motors(l, r, mRegulateSpeed, mSynchronizeMotors);

            mMotorSpeedCalculator = new MotorSpeedCalculator();

        }


        // Process preview frame
        mProcessedFrame = mLaneDetector.getFrameWithLanes(mAccentColor);


//        return mLaneDetector.getTemp();
        return mProcessedFrame;
    }

}