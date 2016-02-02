package tw.blogspot.gaujei.asusbot;

import android.os.Message;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.os.Handler; //for timer
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.ImageButton;
import android.widget.SeekBar;
import android.widget.TextView;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import com.asus.control.ControlAPI;



public class MainActivity extends AppCompatActivity {

    public static String TAG = "ASUS_MainActivity";
    TextView DebugView = null;
    TextView SensorView = null;
    TextView SensorRAW = null;
    ControlAPI mBaseControlAPI = new ControlAPI();


    //for Sensor timer
    private int m_nTime = 0;
    private Handler mHandlerTime = new Handler();
    private Handler mHandlerBase = new Handler();
    private Handler mHandlerHead = new Handler();
    public int [] SensorStatus;

    // For User Tracking and Following
    private float AmpYaw = 20, AmpPitch = 20;   // 10 degrees
    private float PhaseDiff = 0;   // 0 degrees
    private float Frequency = 10;    // 5 Hz
    private float AmpRadius = 1;   // 1 m
    @Override
    // Timer
    // Save current status and continue running status
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main); // assign the layout (R -> class for App project source, many inner class)

        findViews();
        setListenerOnClick();

        mHandlerTime.postDelayed(timerRun, 1000); //for Sensor timer

        mBaseControlAPI.activeSensorWatcher(new ControlAPI.SensorDataListener() {
            @Override
            public void onSenorDataUpdate(int value) {
                //Update ur UI
            }
        });
    }

    @Override
    public void onDestroy() {
        mHandlerTime.removeCallbacks(timerRun); //for Sensor timer
        mHandlerHead.removeCallbacks(null);
        mHandlerBase.removeCallbacks(null);
        mBaseControlAPI.deActiveSensorWatch();
        super.onDestroy();
    }

    //for Sensor timer
    private final Runnable timerRun = new Runnable() {

        public void run() {
            ++m_nTime;
            mHandlerTime.postDelayed(this, 20); //ms

            SensorStatus = mBaseControlAPI.GetSensorData(0x01);
            SensorView.setText("TimeStamp:" + Integer.toString(SensorStatus[0]) + "(msec)"
                    + "\nStatus -> Drop:" + Integer.toString(SensorStatus[1])
                    + ", Sonar:" + Integer.toString(SensorStatus[2])
                    + ", Overcurrent:" + Integer.toString(SensorStatus[3])
                    + ", OverTemp:" + Integer.toString(SensorStatus[4])
                    + ", Neck:" + Integer.toString(SensorStatus[5])
                    + ", Mobile:" + Integer.toString(SensorStatus[6])
                    + ", Power:" + Integer.toString(SensorStatus[7])
                    + ", LeftWheel:" + Integer.toString(SensorStatus[8])
                    + ", RightWheel:" + Integer.toString(SensorStatus[9])
                    + ", NeckYaw:" + Integer.toString(SensorStatus[10])
                    + ", NeckPitch:" + Integer.toString(SensorStatus[11])
                    + "\nTask.x:" + Integer.toString(SensorStatus[12]) + " (mm)"
                    + ", Task.y:" + Integer.toString(SensorStatus[13]) + " (mm)"
                    + ", Task.theta:" + Double.toString(SensorStatus[14]/10) + " (deg)"         // Convert from 0.1deg to deg
                    + "\nAvoidTarget.x:" + Integer.toString(SensorStatus[15]) + " (mm)"
                    + ", AvoidTarget.y:" + Integer.toString(SensorStatus[16]) + " (mm)"
            );

            int [] SensorSonarRAWData = mBaseControlAPI.GetSensorRAWData(0x01); //Sonar
            SensorRAW.setText("Sonar RAW:" + Integer.toString(SensorSonarRAWData[0]) + ", " + Integer.toString(SensorSonarRAWData[1]) + ", " + Integer.toString(SensorSonarRAWData[2]));

/*
            int [] SensorDROPIRRAWData0 = mBaseControlAPI.GetSensorRAWData(0x00); //Drop IR
            SensorView.setText("Drop IR RAW:" + Integer.toString(SensorDROPIRRAWData0[0]) + ", " + Integer.toString(SensorDROPIRRAWData0[1]) + ", " + Integer.toString(SensorDROPIRRAWData0[2]));

            int [] SensorRAWData = mBaseControlAPI.GetSensorRAWData(0x06); //Motor temp
            SensorRAW.setText("Drop IR RAW:" + Integer.toString(SensorRAWData[0]) + ", " + Integer.toString(SensorRAWData[1])
                                      + ", " + Integer.toString(SensorRAWData[2]) + ", " + Integer.toString(SensorRAWData[3]));
*/
        }
    };


    @Override
    public boolean onCreateOptionsMenu(Menu menu) { // Open res/menu/main.xml
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    private View.OnClickListener btnInitialDeviceOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            DebugView.setText(mBaseControlAPI.initialDevice());
        }
    };

    private View.OnClickListener btnChevronDoubleDownOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {

            DebugView.setText(mBaseControlAPI.remoteCtrlBase(mBaseControlAPI.Stop));   // Stop
        }
    };

    private View.OnClickListener btnArrowUpOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {

            DebugView.setText(mBaseControlAPI.remoteCtrlBase(mBaseControlAPI.Forward));   // Forward
        }
    };

    private View.OnClickListener btnArrowDownOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {

            DebugView.setText(mBaseControlAPI.remoteCtrlBase(mBaseControlAPI.Backward));   // Backward
        }
    };

    private View.OnClickListener btnArrowLeftOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {

            DebugView.setText(mBaseControlAPI.remoteCtrlBase(mBaseControlAPI.TurnLeft));   // Left
        }
    };

    private View.OnClickListener btnArrowRightOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {

            DebugView.setText(mBaseControlAPI.remoteCtrlBase(mBaseControlAPI.TurnRight));   // Right
        }
    };


    private View.OnClickListener btnCmdOnClick = new View.OnClickListener(){
        @Override
        public void onClick(View v) {

            int iCmdID = Integer.parseInt(mEdt_CmdID.getText().toString());
            int iCmdSize = 0x06;
            int iCmdData0 = Integer.parseInt(mEdt_CmdData0.getText().toString());
            int iCmdData1 = Integer.parseInt(mEdt_CmdData1.getText().toString());
            int iCmdData2 = Integer.parseInt(mEdt_CmdData2.getText().toString());
            int iCmdData3 = Integer.parseInt(mEdt_CmdData3.getText().toString());
            int iCmdData4 = Integer.parseInt(mEdt_CmdData4.getText().toString());
            int iCmdData5 = Integer.parseInt(mEdt_CmdData5.getText().toString());

            int [] iCmdData = {iCmdData0, iCmdData1, iCmdData2, iCmdData3, iCmdData4, iCmdData5};

            //Log.i(TAG, "Cmd" + Integer.toString(iCmdData[0]) + Integer.toString(iCmdData[1]) + Integer.toString(iCmdData[2]) + Integer.toString(iCmdData[3]) + Integer.toString(iCmdData[4]));

            mBaseControlAPI.sendCommand(iCmdID, iCmdSize, iCmdData);
            //DebugView.setText(mBaseControlAPI.sendCommand(iCmdID, iCmdSize, iCmdData));
        }
    };


    private View.OnClickListener btnNeckOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            float iYawAngle = Float.parseFloat(mEdtYawAngle.getText().toString());
            float iYawTime = Float.parseFloat(mEdtYawTime.getText().toString());
            float iPitchAngle = Float.parseFloat((mEdtPitchAngle.getText().toString()));
            float iPitchTime = Float.parseFloat(mEdtPitchTime.getText().toString());

            DebugView.setText(mBaseControlAPI.ctrlNeckJoint(iYawAngle, iYawTime, iPitchAngle, iPitchTime));
        }
    };

    private View.OnClickListener btnSpecificActionOnClick = new View.OnClickListener(){
        @Override
        public void onClick(View v) {
            final int iItemNo = Integer.parseInt(mEdtSpecificItemNo.getText().toString());

//            DebugView.setText(mBaseControlAPI.controlNeckSpecificAction(iItemNo));
            new Thread (new Runnable() {
                @Override
                public void run() {
                    Message msg = new Message();
                    mBaseControlAPI.controlNeckSpecificAction(iItemNo);
                    try {
                        Thread.sleep(5000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    msg.what = 1;
                    mHandlerBase.sendMessage(msg); // tell the main thread to execute the next function
                }

            }).start();
        }
    };

    // User Tracking
    private View.OnClickListener btnTrackOnClick = new View.OnClickListener(){
        @Override
        public void onClick(View v) {

            final int cycle = 10;   // How many cycles
            final float sampleTime = 0.1f;  // Assume the command is updated by vision team every 100 msec
//            final float ix = Float.parseFloat(mEdtTrackX.getText().toString());
//            final float iy = Float.parseFloat((mEdtTrackY.getText().toString()));

            new Thread( new Runnable() {
                @Override
                public void run() {
                    Message msg = new Message();

                    //DebugView.setText(mBaseControlAPI.relMoveMultiSeg());
                    //final String response = mBaseControlAPI.relMoveMultiSeg();

                    for (int iTimeStamp = 0 ; iTimeStamp< (int)(cycle*(Frequency/sampleTime)) ; iTimeStamp++) {
                        float cvYawAngle, cvPitchAngle;
                        cvYawAngle = (float) (AmpYaw * Math.sin( iTimeStamp*sampleTime/Frequency * 2*Math.PI));
                        cvPitchAngle = (float) (AmpPitch * Math.sin( iTimeStamp*sampleTime/Frequency * 2*Math.PI + PhaseDiff/(2*Math.PI)));
                        mBaseControlAPI.trackUser(cvYawAngle, cvPitchAngle);
                        try {
                            Thread.sleep((int) (sampleTime*1000) );
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    msg.what = 1;
                    mHandlerBase.sendMessage(msg);

                }
            }).start();
        }
    };
    // User Following
    private View.OnClickListener btnFollowOnClick = new View.OnClickListener(){
        @Override
        public void onClick(View v) {

            final int cycle = 10;   // How many cycles
            final float sampleTime = 0.1f;  // Assume the command is updated by vision team every 100 msec
            new Thread( new Runnable() {
                @Override
                public void run() {
                    Message msg = new Message();

                    //DebugView.setText(mBaseControlAPI.relMoveMultiSeg());
                    //final String response = mBaseControlAPI.relMoveMultiSeg();

                    for (int iTimeStamp = 0 ; iTimeStamp< (int)(cycle*(Frequency/sampleTime)) ; iTimeStamp++) {

                        float cvYawAngle = (float) (AmpYaw * Math.sin( iTimeStamp*sampleTime/Frequency * 2*Math.PI));
                        float cvPitchAngle = (float) (AmpPitch * Math.sin( iTimeStamp*sampleTime/Frequency * 2*Math.PI + PhaseDiff/(2*Math.PI)));
                        float cvRadius = (float) (AmpRadius * Math.sin( iTimeStamp*sampleTime/Frequency * 2*Math.PI)) + 1;
//                        float cvRadius = 0.5f;
                        mBaseControlAPI.followUser(cvYawAngle, cvPitchAngle, cvRadius);

                        try {
                            Thread.sleep((int) (sampleTime*1000) );
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    msg.what = 1;
                    mHandlerBase.sendMessage(msg);

                }
            }).start();
        }
    };

//    SeekBar.OnSeekBarChangeListener  changeTrackYaw = new SeekBar.OnSeekBarChangeListener() {
//
//        @Override
//        public void onStopTrackingTouch(SeekBar seekBar) {
//            // TODO Auto-generated method stub
//        }
//
//        @Override
//        public void onStartTrackingTouch(SeekBar seekBar) {
//            // TODO Auto-generated method stub
//        }
//
//        @Override
//        public void onProgressChanged(SeekBar v, int progress, boolean isUser) {
//            // TODO Auto-generated method stub
//            float UserTrackTarget_Yaw = (float) (progress - 45);
//
//            mBaseControlAPI.mTarget_Yaw = UserTrackTarget_Yaw;
//            mTxtUserTrackYaw.setText(Float.toString(UserTrackTarget_Yaw) + " cm");
//        }
//    };


    private View.OnClickListener btnCalculateOnClick = new View.OnClickListener(){
        @Override
        public void onClick(View v) {
            final float ixA = Float.parseFloat(mEdt_xA.getText().toString());   // Integer.parseInt
            final float iyA = Float.parseFloat(mEdt_yA.getText().toString());
            final float ithetaA = Float.parseFloat(mEdt_thetaA.getText().toString());
            final float ixB = Float.parseFloat(mEdt_xB.getText().toString());
            final float iyB = Float.parseFloat(mEdt_yB.getText().toString());
            final float ithetaB = Float.parseFloat(mEdt_thetaB.getText().toString());

            Log.i(TAG, "A* path-finding calculation for (" + ixA + "," + iyA + "," + ixB + "," + iyB + ")");
            //DebugView.setText(mBaseControlAPI.goFromAToB(ixA, iyA, ithetaA, ixB, iyB, ithetaB));

            new Thread( new Runnable() {
                @Override
                public void run() {
                    Message msg = new Message();

                    //DebugView.setText(mBaseControlAPI.relMoveMultiSeg());
                    //final String response = mBaseControlAPI.relMoveMultiSeg();
                    mBaseControlAPI.goFromAToB(ixA, iyA, ithetaA, ixB, iyB, ithetaB);
                    msg.what = 1;
                    mHandlerBase.sendMessage(msg);
                }
            }).start();

            mHandlerBase = new Handler(){
                @Override
                public void handleMessage(Message msg) {
                    switch(msg.what){
                        case 1:
                            DebugView.setText("Success!");
                            break;
                    }
                }
            };
        }
    };


    private View.OnClickListener btnRelMoveToOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            final float iRelX = Float.parseFloat(mEdt_relX.getText().toString());
            final float iRelY = Float.parseFloat(mEdt_relY.getText().toString());
            final float iRelTheta = Float.parseFloat(mEdt_relTheta.getText().toString());

            // DebugView.setText(mBaseControlAPI.relMoveTo(iRelX, iRelY, iRelTheta));
//            MyThread mythread1 = new MyThread();
//            mythread1.start();

            new Thread( new Runnable() {
                @Override
                public void run() {
                    Message msg = new Message();

                    //DebugView.setText(mBaseControlAPI.relMoveMultiSeg());
                    //final String response = mBaseControlAPI.relMoveMultiSeg();
                    mBaseControlAPI.relMoveTo(iRelX, iRelY, iRelTheta);
                    msg.what = 1;
                    mHandlerBase.sendMessage(msg);
                }
            }).start();


        }
    };

    private View.OnClickListener btnRelMoveToCylindricalOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            float iRho = Float.parseFloat(mEdt_rho.getText().toString());
            float iAlpha = Float.parseFloat(mEdt_alpha.getText().toString());
            float iBeta = Float.parseFloat(mEdt_beta.getText().toString());

            DebugView.setText(mBaseControlAPI.relMoveToCylindrical(iRho, iAlpha, iBeta));
        }
    };



    private View.OnClickListener btnBikeOnClick = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            int iRho = Integer.parseInt(mEdt_rho.getText().toString());
            int iAlpha = Integer.parseInt(mEdt_alpha.getText().toString());
            int iBeta = Integer.parseInt(mEdt_beta.getText().toString());

            new Thread( new Runnable() {
                @Override
                public void run() {
                    Message msg = new Message();

                    //DebugView.setText(mBaseControlAPI.relMoveMultiSeg());
                    //final String response = mBaseControlAPI.relMoveMultiSeg();
                    mBaseControlAPI.relMoveMultiSeg();
                    msg.what = 1;
                    mHandlerBase.sendMessage(msg);
                }
            }).start();

            mHandlerBase = new Handler(){
                @Override
                public void handleMessage(Message msg) {
                    switch(msg.what){
                        case 1:
                            DebugView.setText("Success!");
                            break;
                    }
                }
            };
        }
    };

    SeekBar.OnSeekBarChangeListener  changeLeftSonar = new SeekBar.OnSeekBarChangeListener() {

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
            // TODO Auto-generated method stub
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {
            // TODO Auto-generated method stub
        }

        @Override
        public void onProgressChanged(SeekBar v, int progress, boolean isUser) {
            // TODO Auto-generated method stub
            int iCmdID = 0x28;
            int iCmdSize = 0x02;
            int iSonarNo = 0x00;    // front left sonar

            int [] iCmdData = {iSonarNo, progress};

            // DebugView.setText(mBaseControlAPI.sendCommand(iCmdID, iCmdSize, iCmdData));
            if (mBaseControlAPI.sendCommand(iCmdID, iCmdSize, iCmdData) >= 0){
                DebugView.setText("Left sonar value is changed!");
            }
            mTxtLeftSonar.setText(Integer.toString(progress) + " cm");
        }
    };

    SeekBar.OnSeekBarChangeListener  changeRightSonar = new SeekBar.OnSeekBarChangeListener() {

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
            // TODO Auto-generated method stub
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {
            // TODO Auto-generated method stub
        }

        @Override
        public void onProgressChanged(SeekBar v, int progress, boolean isUser) {
            // TODO Auto-generated method stub
            int iCmdID = 0x28;
            int iCmdSize = 0x02;
            int iSonarNo = 0x01;    // front right sonar
            int [] iCmdData = {iSonarNo, progress};

            if (mBaseControlAPI.sendCommand(iCmdID, iCmdSize, iCmdData) >= 0){
                DebugView.setText("Right sonar value is changed!");
            }
            mTxtRightSonar.setText(Integer.toString(progress) + " cm");
        }
    };

    SeekBar.OnSeekBarChangeListener  changeRearSonar = new SeekBar.OnSeekBarChangeListener() {

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
            // TODO Auto-generated method stub
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {
            // TODO Auto-generated method stub
        }

        @Override
        public void onProgressChanged(SeekBar v, int progress, boolean isUser) {
            // TODO Auto-generated method stub
            int iCmdID = 0x28;
            int iCmdSize = 0x02;
            int iSonarNo = 0x02;    // front right sonar
            int [] iCmdData = {iSonarNo, progress};

            if (mBaseControlAPI.sendCommand(iCmdID, iCmdSize, iCmdData) >= 0){
                DebugView.setText("Rear sonar value is changed!");
            }
            mTxtRearSonar.setText(Integer.toString(progress) + " cm");
        }
    };

    /**
     * Define Buttons, TextView, EditText
     */
    private Button mBtnInitialDevice;
    private Button mBtnForward, mBtnBack, mBtnLeft, mBtnRight, mBtnStop;

    //Command Package Input
    private ImageButton mBtnArrowLeft, mBtnArrowRight, mBtnArrowUp, mBtnArrowDown, mBtnChevronDoubleDown;

    private EditText mEdt_CmdID, mEdt_CmdData0, mEdt_CmdData1, mEdt_CmdData2, mEdt_CmdData3, mEdt_CmdData4, mEdt_CmdData5;
    private Button mBtnCommandSend;

    private EditText mEdtYawAngle, mEdtYawTime, mEdtPitchAngle, mEdtPitchTime;
    private Button mBtnNeck;

    // Define User tracking Button
//    private EditText mEdtTrackX, mEdtTrackY;
//    private TextView mTxtUserTrackYaw;
    private Button mBtnTrack;
    private Button mBtnFollow;
//    private SeekBar mBarUserTrackYaw;

    private EditText mEdt_relX, mEdt_relY, mEdt_relTheta;
    private Button mBtnRelMoveTo;

    private EditText mEdtSpecificItemNo;
    private Button mBtnSpecificAction;

    private EditText mEdt_rho, mEdt_alpha, mEdt_beta;
    private Button mBtnRelMoveToCylindrical;

    private EditText mEdt_xA, mEdt_yA, mEdt_thetaA, mEdt_xB, mEdt_yB, mEdt_thetaB;
    private Button mBtnCalculate;

    private ImageButton mBtnBike;
    private SeekBar mBarLeftSonar, mBarRightSonar, mBarRearSonar;
    private TextView mTxtLeftSonar, mTxtRightSonar, mTxtRearSonar;


    //Connect to the Button and Text

    private void findViews() {
        DebugView = (TextView) this.findViewById(R.id.TextViewDebug);
        SensorView = (TextView) this.findViewById(R.id.TextViewSensorData);
        SensorRAW = (TextView) this.findViewById(R.id.TextViewSensorRAW);

        mBtnInitialDevice = (Button) this.findViewById(R.id.btnInitialDevice);
        mBtnForward = (Button) this.findViewById(R.id.btnForward);
        mBtnBack = (Button) this.findViewById(R.id.btnBack);
        mBtnLeft = (Button) this.findViewById(R.id.btnLeft);
        mBtnRight = (Button) this.findViewById(R.id.btnRight);
        mBtnStop = (Button) this.findViewById(R.id.btnStop);

        mEdt_CmdID = (EditText) this.findViewById(R.id.edt_CmdID);
        mEdt_CmdData0 = (EditText) this.findViewById(R.id.edt_CmdData0);
        mEdt_CmdData1 = (EditText) this.findViewById(R.id.edt_CmdData1);
        mEdt_CmdData2 = (EditText) this.findViewById(R.id.edt_CmdData2);
        mEdt_CmdData3 = (EditText) this.findViewById(R.id.edt_CmdData3);
        mEdt_CmdData4 = (EditText) this.findViewById(R.id.edt_CmdData4);
        mEdt_CmdData5 = (EditText) this.findViewById(R.id.edt_CmdData5);
        mBtnCommandSend = (Button) this.findViewById(R.id.CB_Send);

        mBtnArrowLeft = (ImageButton) this.findViewById(R.id.btnArrowLeft);
        mBtnArrowRight = (ImageButton) this.findViewById(R.id.btnArrowRight);
        mBtnArrowUp = (ImageButton) this.findViewById(R.id.btnArrowUp);
        mBtnArrowDown = (ImageButton) this.findViewById(R.id.btnArrowDown);
        mBtnChevronDoubleDown = (ImageButton) this.findViewById(R.id.btnChevronDoubleDown);

        mEdtYawAngle = (EditText) this.findViewById(R.id.edtYawAngle);
        mEdtYawTime = (EditText) this.findViewById(R.id.edtYawTime);
        mEdtPitchAngle = (EditText) this.findViewById(R.id.edtPitchAngle);
        mEdtPitchTime = (EditText) this.findViewById(R.id.edtPitchTime);
        mBtnNeck = (Button) this.findViewById(R.id.btnNeck);

        // User tracking
//        mEdtTrackX = (EditText) this.findViewById(R.id.edtTrackYaw);
//        mEdtTrackY = (EditText) this.findViewById(R.id.edtTrackPitch);
        mBtnTrack = (Button) this.findViewById(R.id.btnTrack);
        mBtnFollow = (Button) this.findViewById(R.id.btnFollow);
//        mBarUserTrackYaw = (SeekBar)this.findViewById(R.id.barUserTrackYaw);
//        mBarUserTrackYaw.setMax(90);
//        mBarUserTrackYaw.setProgress(45);
//        mTxtUserTrackYaw = (TextView) this.findViewById(R.id.txtUserTrackYaw);

        mEdtSpecificItemNo = (EditText) this.findViewById(R.id.edtSpecificItemNo);
        mBtnSpecificAction = (Button) this.findViewById(R.id.btnSpecificAction);

        mEdt_relX = (EditText) this.findViewById(R.id.edt_relX);
        mEdt_relY = (EditText) this.findViewById(R.id.edt_relY);
        mEdt_relTheta = (EditText) this.findViewById(R.id.edt_relTheta);
        mBtnRelMoveTo = (Button) this.findViewById(R.id.btnRelMoveTo);

        mEdt_rho = (EditText) this.findViewById(R.id.edt_rho);
        mEdt_alpha = (EditText) this.findViewById(R.id.edt_alpha);
        mEdt_beta = (EditText) this.findViewById(R.id.edt_beta);
        mBtnRelMoveToCylindrical = (Button) this.findViewById(R.id.btnRelMoveToCylindrical);

        mEdt_xA = (EditText) this.findViewById(R.id.edt_xA);
        mEdt_yA = (EditText) this.findViewById(R.id.edt_yA);
        mEdt_thetaA = (EditText) this.findViewById(R.id.edt_thetaA);
        mEdt_xB = (EditText) this.findViewById(R.id.edt_xB);
        mEdt_yB = (EditText) this.findViewById(R.id.edt_yB);
        mEdt_thetaB = (EditText) this.findViewById(R.id.edt_thetaB);
        mBtnCalculate = (Button) this.findViewById(R.id.btnCalculate);

        mBtnBike = (ImageButton) this.findViewById(R.id.btnBike);
        mBarLeftSonar = (SeekBar)findViewById(R.id.barLeftSonar);
        mBarLeftSonar.setMax(100);
        mBarLeftSonar.setProgress(70);
        mTxtLeftSonar = (TextView) this.findViewById(R.id.txtLeftSonar);
        mBarRightSonar = (SeekBar)findViewById(R.id.barRightSonar);
        mBarRightSonar.setMax(100);
        mBarRightSonar.setProgress(70);
        mTxtRightSonar = (TextView) this.findViewById(R.id.txtRightSonar);
        mBarRearSonar = (SeekBar)findViewById(R.id.barRearSonar);
        mBarRearSonar.setMax(100);
        mBarRearSonar.setProgress(70);
        mTxtRearSonar = (TextView) this.findViewById(R.id.txtRearSonar);
    }

    private void setListenerOnClick() {
        mBtnInitialDevice.setOnClickListener(btnInitialDeviceOnClick);

        mBtnCommandSend.setOnClickListener(btnCmdOnClick);

        mBtnArrowLeft.setOnClickListener(btnArrowLeftOnClick);
        mBtnArrowRight.setOnClickListener(btnArrowRightOnClick);
        mBtnArrowUp.setOnClickListener(btnArrowUpOnClick);
        mBtnArrowDown.setOnClickListener(btnArrowDownOnClick);
        mBtnChevronDoubleDown.setOnClickListener(btnChevronDoubleDownOnClick);

        mBtnNeck.setOnClickListener(btnNeckOnClick);
        // Call Function
        mBtnTrack.setOnClickListener(btnTrackOnClick);
        mBtnFollow.setOnClickListener(btnFollowOnClick);
//        mBarUserTrackYaw.setOnSeekBarChangeListener(changeTrackYaw);

        mBtnSpecificAction.setOnClickListener(btnSpecificActionOnClick);

        mBtnRelMoveTo.setOnClickListener(btnRelMoveToOnClick);

        mBtnRelMoveToCylindrical.setOnClickListener(btnRelMoveToCylindricalOnClick);

        mBtnCalculate.setOnClickListener(btnCalculateOnClick);

        mBtnBike.setOnClickListener(btnBikeOnClick);

        mBarLeftSonar.setOnSeekBarChangeListener(changeLeftSonar);
        mBarRightSonar.setOnSeekBarChangeListener(changeRightSonar);
        mBarRearSonar.setOnSeekBarChangeListener(changeRearSonar);
    }

//    public class MyThread extends Thread{
//        private Handler mHandler;
//        public void run(){
//            final float iRelX = Float.parseFloat(mEdt_relX.getText().toString());
//            final float iRelY = Float.parseFloat(mEdt_relY.getText().toString());
//            final float iRelTheta = Float.parseFloat(mEdt_relTheta.getText().toString());
//            mBaseControlAPI.relMoveTo(iRelX, iRelY, iRelTheta);
//            System.out.println("fuck");
//
//        }
//    }

}
