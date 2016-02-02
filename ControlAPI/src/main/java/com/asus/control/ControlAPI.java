package com.asus.control;


import android.content.ClipData;
import android.os.Handler;
import android.os.Message;
import android.util.Log;

import com.asus.control.Constant.e_Base;
import com.asus.control.Constant.e_CoreSensor;
import com.asus.control.Constant.e_Specific;
import com.asus.control.Constant.e_rtn;

import java.io.File;
import java.lang.ref.WeakReference;
import java.util.Scanner;
import java.util.concurrent.TimeUnit;

/**
 * This API provides a interface to transfer command to Linux tty driver.
 */
public class ControlAPI extends Thread {
    public static String TAG = "ASUS_ControlAPI";

    static {
        System.loadLibrary("Asusdriver");
        Log.i(TAG, "Finish load Library......");
    }

    // native private String stringFromJNI();
    private native String NativeSetControlParameter(int cmd[]);
    private native int[] getStatus(int para);
    private native int[] getSensorRAWData(int Type);

    // Motion control for the neck joint and the wheel base
    public native int NativeSendCommand(int CmdID, int CmdSize, int CmdData[]);
    public native int NativeWheelSpdCtrl(int WheelSpd[]);
    public native int NativeReloadMapFile();                         // Initialization: Load the specific-action file
    public native int NativeControlNeckJoint(int targetPose[]);      // NativeSetControlParameter(cmd);
    public native int NativeCtrlTrkNeckJoint(int targetPose[]);
    public native int NativeRelMoveTo(int Volume, int Index, int targetPose[]);
    public native int[] NativeGoFromAToB(int initPose[], int targetPose[]);
    public native int NativePointSmooth(int targetPose[]); // SpecificAction


    // System initial
    //static final int INITAIAL = 0x00;

    // Mobile base remote control
    static final int BASECONTROL = 0x01;
    static int mobileBase_vx = 0, mobileBase_wz = 0, vx = 0, wz = 0;
    public final static int Stop = 0, Forward = 1, Backward = 2, TurnLeft = 3, TurnRight = 4;

    // Request feedback sensor RAW data type
    static final int REQUESTENSORTYPE = 0x08;
    static final int receivedSensorDataSize = 17;
    public int [] SensorData = new int [receivedSensorDataSize];

    //
    private int AvoidTargetX, AvoidTargetY;

    // User tracking
    public float mTarget_Yaw;
    public float mTarget_Pitch;

    private int count = 0;

    private SensorWatcher mSensorWatcher;
    private Thread mSensorWatcherThread;
    private boolean mStopmSensorWatcherThread = false;
    private Handler mUpdateHandler;
    private int YawAngle = 0, PitchAngle = 0;
    private int flag = 0;  // raise flag if triggered

    /**
     * activate the sensor watcher function for watch the status of sensors/motors
     * @param senseDataListener  sensor data
     */
    public void activeSensorWatcher(final SensorDataListener senseDataListener) {
        if (mSensorWatcherThread == null) {
            mUpdateHandler = new Handler() {
                @Override
                public void handleMessage(Message msg) {
                    super.handleMessage(msg);
                    SensorData motorData = (SensorData)msg.obj;
                    if (senseDataListener != null) {
                        senseDataListener.onSenorDataUpdate(motorData.getMotorData());
                    }
                }
            };
            mSensorWatcher = new SensorWatcher(this);
            mSensorWatcherThread = new Thread(mSensorWatcher);
            mSensorWatcherThread.start();
            mStopmSensorWatcherThread = false;
        }
    }

    public void deActiveSensorWatch() {
        if (mSensorWatcherThread != null) {
            mStopmSensorWatcherThread = true;
            mSensorWatcherThread.interrupt();
            mUpdateHandler.removeCallbacks(null);
            mSensorWatcher = null;
        }
    }


    /**
     * Send out an ASCII command/data to the micro-controller for specific purpose
     * @param CommandID command ID
     * @param CommandData0 command data 0
     * @param CommandData1 command data 1
     * @param CommandData2 command data 2
     * @param CommandData3 command data 3
     * @param CommandData4 command data 4
     * @param CommandData5 command data 5
     * @return true/false
     */
    public String CommandSetInterface(int CommandID, int CommandData0, int CommandData1, int CommandData2, int CommandData3, int CommandData4, int CommandData5) {
        int cmd[] = {CommandID, CommandData0, CommandData1, CommandData2, CommandData3, CommandData4, CommandData5, 7, 8, 9, 10};
        String result = NativeSetControlParameter(cmd);
        return result;
    }


    /**
     * Initialize the ttyACM0 device to update sensor date/status.
     * @ param : void
     * @ return: true (successful)
     */
    public String initialDevice() //initial
    {
        // System initial
        int INITIAL = 0x00;
        mobileBase_vx = 0;
        mobileBase_wz = 0;
        int cmd[] = {INITIAL, 0, 0, 0, 0, 5, 6, 7, 8, 9, 10}; //initial
        String result = NativeSetControlParameter(cmd);
        Log.i(TAG, "API -> initialDevice");

        return result;
    }


    /**
     * This command switch is send a request to get sensor raw data from MCU.
     * Bit 0: Drop IR RAW Data
     * Bit 1: Sonar IR RAW Data
     * Bit 2: Docking
     * Bit 3: G sensor
     * Bit 4: Gyro
     * Bit 5: E-compass
     * Bit 6: Motor temperature
     */
    public String RequestSensorRAWDate(int type) {

        int cmd[] = {REQUESTENSORTYPE, type, 0, 0, 0, 5, 6, 7, 8, 9, 10};
        String result = NativeSetControlParameter(cmd);
        Log.i(TAG, "API -> VelocityControl");
        return result;
    }


    /**
     * get some pre-defined Sensor information and robot's status (This function will not be provided in the future.)
     * @param DataNo 0x01 by default
     * @return raw data in array
     */
    public int [] GetSensorData(int DataNo) {

        int[] x ;
        x = getStatus(DataNo);
        //Log.i(TAG, "API -> GetSensorData");

        for(int i = 0; i < receivedSensorDataSize ; i++) {
            SensorData[i] = x[i];
        }
        return SensorData;
    }


    /**
     * get the Sensor's raw data based on the input "Type" (This function will not be provided in the future.)
     * @param Type 0x00: Drop IR[3], 0x01:Sonar[3], 0x02:Docking IR[2], 0x03:G sensor[3], 0x04:Gyro sensor[3], 0x05:E-compass[3], 0x06:Motor temperature[4],
     * @return raw data in array
     */
    public int [] GetSensorRAWData(int Type) {

        int[] x ;
        int size = 0;
        x = getSensorRAWData(Type);


        if(Type == 0x00){
            size = 3;
        }
        else if(Type == 0x01){
            size = 3;
        }
        else if(Type == 0x02){
            size = 2;
        }
        else if(Type == 0x03){
            size = 3;
        }
        else if(Type == 0x04){
            size = 3;
        }
        else if(Type == 0x05){
            size = 3;
        }
        else if(Type == 0x06){
            size = 4;
        }
        else
        {
            size = 0;
        }

        int [] RAW = new int[size];
        for(int i = 0; i < size ; i++) {
            RAW[i] = x[i];
        }
        return RAW;
    }


    /**
     * Send a command given by user to MCU
     * @param CmdID
     * @param CmdSize
     * @param CmdData
     * @return
     */
    public int sendCommand(int CmdID, int CmdSize, int CmdData[]) {
        int result = NativeSendCommand(CmdID, CmdSize, CmdData);
        return result;
    }

    /**
     * Reload the map file, map.txt, if the map is re-built (Theoretically, it will be called by QianHao)
     * @param
     * @return
     */
    private String reloadMapFile() {

        return ("Reload Map : " + Integer.toString( NativeReloadMapFile() ) );
    }

    /**
     * Control Neck Position in joint space
     * @param   pitchAngle  rotational angle in pitch direction (degree)
     * @param   pitchTime   time (sec)
     * @param   yawAngle    rotational angle in yaw direction (degree)
     * @param   yawTime     time (sec)
     * @return true/false
     */
    public String ctrlNeckJoint(float yawAngle, float yawTime, float pitchAngle, float pitchTime) {

        int targetPose[] = {(int) yawAngle*10, (int) pitchAngle*10, (int) (yawTime*1000), (int) (pitchTime*1000)};  // Convert to 0.1 deg and msec
        int result = NativeControlNeckJoint(targetPose);   // NativeSetControlParameter(cmd);
//        Log.i("targetPose : ", Integer.toString(targetPose[0]) + "," + Integer.toString(targetPose[1]) +"," + Integer.toString(targetPose[2]));
        return ("Control Neck Position : " + Integer.toString( result ) );
    }

    public String ctrlNeckJointTrk(float yawAngle, float yawTime, float pitchAngle, float pitchTime) {

        int targetPose[] = {(int) (yawAngle*10), (int) (pitchAngle*10), (int) (yawTime*1000), (int) (pitchTime*1000)};  // Convert to 0.1 deg and msec
        int result = NativeCtrlTrkNeckJoint(targetPose);   // NativeSetControlParameter(cmd);
        return ("Control Neck Position : " + Integer.toString( result ) );
    }

    /**
     * Control head position in task space (Calculated based on inverse kinematics)
     * @param x absolute position of the center of the head in x-axis direction (m)
     * @param y absolute position of the center of the head in y-axis direction (m)
     * @param time time (sec)
     * @return true/false
     */
    public String ctrlNeckTask(float x, float y, float time) {
        return "true";
    }


    /**
     * Perform the specific action for head rotation in yaw and pitch directions
     * @param   ItemNo   Item No. of the specific action
     * @return true/false
     */
    public String controlNeckSpecificAction(int ItemNo) {
        int row, col, rowSize = 9, colSize = 100; // Size of the file
        int WaitTime, result, Cycles = 0, bufTime = 100, count = 0;
        int[][] ActionArray = new int[rowSize][colSize];
        int[] targetPose = {0, 0, 0, 0};
        int oldPitchAngle = PitchAngle, oldYawAngle = YawAngle;     // For the purpose of reversing to original posture
        int[] SensorStatus = GetSensorData(0x01);
        int[] JointAngle = {YawAngle, PitchAngle};
        short pitchEncAfter = (short) (SensorStatus[e_CoreSensor.NeckPitchEnc]);
        short yawEncAfter = (short) (SensorStatus[e_CoreSensor.NeckYawEnc]);
        float jointYawAfter = (float) (yawEncAfter);      // Convert encoder count to degree
        float jointPitchAfter = (float) (pitchEncAfter);
        float[] JointAfter = {(float) (SensorStatus[e_CoreSensor.NeckYawEnc]), (float) (SensorStatus[e_CoreSensor.NeckPitchEnc])};
        StringBuffer sb = new StringBuffer();

        // Load file, SpecificAction.txt, and insert into the array
        try {
            row = 0;
            Scanner inFile = new Scanner(new File("/sdcard/SpecificAction.txt")); // point to the directory
            while (inFile.hasNextLine()) {    // Is there more data to process? return true if there is a line terminator in the input
                col = 0;
                String Line = inFile.nextLine();
                Scanner inLine = new Scanner(Line);
                while (inLine.hasNextInt()) {
                    ActionArray[row][col] = inLine.nextInt();
                    sb.append(ActionArray[row][col] + ", ");
                    //Log.i(TAG, "[" + Integer.toString(row) + " , " + Integer.toString(col) + " ] = " + Integer.toString(ActionArray[row][col]));
                    col++;
                }
                sb.append("\n");
                row++;
            }
            Log.i(TAG, String.valueOf(sb));
            inFile.close();
        } catch (Exception e) {
            targetPose = new int[] {0,2000,0,2000};
            result = NativePointSmooth(targetPose);
            e.printStackTrace();
            Log.i(TAG, "Scan failed");
//             return result;
        }
        int indexAngle = ActionArray[e_Specific.IncAbs][ItemNo];

        do {
            Log.i(TAG, "*************** Start the Specific Action : " + Integer.toString(ItemNo) + " ****************");
            // Repeat the above process, if the ActionArray[e_Specific.Cycle][ItemNo] > 0
            Log.i(TAG, "**(ActionArray[Cycle][ItemNo]+1) : " + Integer.toString((ActionArray[e_Specific.Cycle][ItemNo] + 1)) + ", " + Integer.toString(e_Specific.Cycle) + ", " + Integer.toString(ItemNo));
            for (int loop = 0 ; loop < (ActionArray[e_Specific.Cycle][ItemNo]+1) ; loop++) {
//                if (ActionArray[e_Specific.IncAbs][ItemNo] == 0) {
//                    YawAngle += ActionArray[e_Specific.YawAngle][ItemNo];
//                    PitchAngle += ActionArray[e_Specific.PitchAngle][ItemNo];
//                } else {
//                    YawAngle = ActionArray[e_Specific.YawAngle][ItemNo];
//                    PitchAngle = ActionArray[e_Specific.PitchAngle][ItemNo];
//
//                }

//                 0: all rel, 1: Yaw->abs,Pitch->rel, 2: Yaw->rel,Pitch->abs, 3: all abs
                if ((indexAngle & 0x00000001) == 1) {
                    YawAngle = ActionArray[e_Specific.YawAngle][ItemNo];
                } else {
                    YawAngle += ActionArray[e_Specific.YawAngle][ItemNo];
                }
                if (((indexAngle >> 1) & 0x00000001) == 1) {
                    PitchAngle = ActionArray[e_Specific.PitchAngle][ItemNo];
                } else {
                    PitchAngle += ActionArray[e_Specific.PitchAngle][ItemNo];
                }
                // Send out the command in unit [0.1deg, 0.1deg, msec, msec]
                targetPose = new int[]{YawAngle * 10, ActionArray[e_Specific.Duration][ItemNo], PitchAngle * 10, ActionArray[e_Specific.Duration][ItemNo]};  // Convert to 0.1 deg and msec
                result = NativePointSmooth(targetPose);
                JointAngle = new int[]{YawAngle, PitchAngle};
                pitchEncAfter = (short) (SensorStatus[e_CoreSensor.NeckPitchEnc]);
                yawEncAfter = (short) (SensorStatus[e_CoreSensor.NeckYawEnc]);
                jointYawAfter = (float) (yawEncAfter) / 4096 * 360;      // Convert encoder count to degree
                jointPitchAfter = (float) (pitchEncAfter) / 4096 * 360;
                do {
                    Log.i(TAG, "Check Status in first move");
                    pitchEncAfter = (short) (SensorStatus[e_CoreSensor.NeckPitchEnc]);
                    yawEncAfter = (short) (SensorStatus[e_CoreSensor.NeckYawEnc]);
                    jointYawAfter = (float) (yawEncAfter) / 4096 * 360;      // Convert encoder count to degree
                    jointPitchAfter = (float) (pitchEncAfter) / 4096 * 360;
                    JointAfter = new float[]{jointYawAfter, jointPitchAfter};
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    count++;
                }
                while (((Math.abs(JointAngle[0] - JointAfter[0]) >= 1) || (Math.abs(JointAngle[1] - JointAfter[1]) >= 1)) && (count < 1.1 * ActionArray[e_Specific.Duration][ItemNo] / 100));
                count = 0;
                // Reverse to the original posture
                if (ActionArray[e_Specific.ReverseTime][ItemNo] > 0) {

                    PitchAngle = oldPitchAngle;
                    YawAngle = oldYawAngle;
                    // Send out the command in unit [0.1deg, 0.1deg, msec, msec]
                    targetPose = new int[]{YawAngle * 10, ActionArray[e_Specific.ReverseTime][ItemNo], PitchAngle * 10, ActionArray[e_Specific.ReverseTime][ItemNo]};  // Convert to 0.1 deg and msec
                    result = NativePointSmooth(targetPose);

                    // Wait for a while based on "ReverseTime".  Note that "bufTime" msec is appended for guaranteeing the motion is finished,
                    do {
                        Log.i(TAG, "Check Status in Reverse");
                        pitchEncAfter = (short) (SensorStatus[e_CoreSensor.NeckPitchEnc]);
                        yawEncAfter = (short) (SensorStatus[e_CoreSensor.NeckYawEnc]);
                        jointYawAfter = (float) (yawEncAfter) / 4096 * 360;      // Convert encoder count to degree
                        jointPitchAfter = (float) (pitchEncAfter) / 4096 * 360;
                        JointAfter = new float[]{jointYawAfter, jointPitchAfter};
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        count++;
                    }
                    while (((Math.abs(JointAngle[0] - JointAfter[0]) >= 1) || (Math.abs(JointAngle[1] - JointAfter[1]) >= 1)) && (count < 1.1 * ActionArray[e_Specific.Duration][ItemNo] / 100));
                    count = 0;
                    Log.i(TAG, "ReverseTime = " + Integer.toString(ActionArray[e_Specific.ReverseTime][ItemNo]));
                    Log.i(TAG, "Reverse to the original posture: [ " + Integer.toString(targetPose[0]) + " , " + Integer.toString(targetPose[1]) + " , " + Integer.toString(targetPose[2]) + " ]");
                }
            }
            //Next-step motion, if necessary

            if (ActionArray[e_Specific.NextMotionDelay][ItemNo] >= 0) {

                try {
                    WaitTime = ActionArray[e_Specific.NextMotionDelay][ItemNo] + bufTime;
                    Thread.sleep(WaitTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                // Next-step motion //e_NextMotion
                ItemNo = ActionArray[e_Specific.NextMotion][ItemNo];
                Cycles++;
            } else {
                ctrlNeckJoint(YawAngle,1000,PitchAngle,1000);
                Cycles = 0;
            }
            Log.i(TAG, "e_NextMotionDelay" + Integer.toString(ActionArray[e_Specific.NextMotionDelay][ItemNo]) + " ; e_NextMotion = " + Integer.toString(ActionArray[e_Specific.NextMotion][ItemNo]));
            Log.i(TAG, "Cycle = " + Integer.toString(Cycles) + " ; ItemNo = " + Integer.toString(ItemNo));
        } while (Cycles > 0 && Cycles < 30);    // Limit the cycles less than 4

        result = 1;
        Log.i(TAG, "API -> Finish Specific Action No. " + Integer.toString(ItemNo));
        return ("Control Neck SpecificAction : " + Integer.toString(result));


//            if (ActionArray[e_Specific.ReverseTime][ItemNo] > 0) {
//                if(ActionArray[e_Specific.NextMotionDelay][ItemNo] >= 0){
//                    targetPose = new int[]{YawAngle * 10, PitchAngle * 10, ActionArray[e_Specific.ReverseTime][ItemNo], ActionArray[e_Specific.ReverseTime][ItemNo]};  // Convert to 0.1 deg and msec
//                    result = NativeControlNeckJoint(targetPose);
//                }else {
//                    targetPose = new int[]{YawAngle * 10, PitchAngle * 10, ActionArray[e_Specific.ReverseTime][ItemNo], ActionArray[e_Specific.ReverseTime][ItemNo]};  // Convert to 0.1 deg and msec
//                    result = NativeControlNeckJoint(targetPose);
//                }
//                PitchAngle = oldPitchAngle;
//                YawAngle = oldYawAngle;
//                // Send out the command in unit [0.1deg, 0.1deg, msec, msec]
//                targetPose = new int[]{YawAngle * 10, PitchAngle * 10, ActionArray[e_Specific.ReverseTime][ItemNo], ActionArray[e_Specific.ReverseTime][ItemNo]};  // Convert to 0.1 deg and msec
//                result = NativeControlNeckJoint(targetPose);
//
//                // Wait for a while based on "ReverseTime".  Note that "bufTime" msec is appended for guaranteeing the motion is finished,
//                do {
//                    try {
//                        Thread.sleep(100);
//                    } catch (InterruptedException e) {
//                        e.printStackTrace();
//                    }
//                    count++;
//                } while ((count < 10) && (SensorStatus[e_CoreSensor.NeckStatus] & 0x03) > 0);
//                try {
//                    WaitTime = ActionArray[e_Specific.PauseTime][ItemNo] + bufTime;
//                    Thread.sleep(WaitTime);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//
//                Log.i(TAG, "ReverseTime = " + Integer.toString(ActionArray[e_Specific.ReverseTime][ItemNo]));
//                Log.i(TAG, "Reverse to the original posture: [ " + Integer.toString(targetPose[0]) + " , " + Integer.toString(targetPose[1]) + " , " + Integer.toString(targetPose[2]) + " ]");
//            }else{
//                targetPose = new int[]{YawAngle * 10, PitchAngle * 10, ActionArray[e_Specific.ReverseTime][ItemNo], ActionArray[e_Specific.ReverseTime][ItemNo]};  // Convert to 0.1 deg and msec
//                result = NativeControlNeckJoint(targetPose);
//            }
//
//            if (ActionArray[e_Specific.NextMotionDelay][ItemNo] >= 0) {
//
//                int [] JointAngle = {YawAngle, PitchAngle};
//                // Send out the command in unit [0.1deg, 0.1deg, msec, msec]
//                targetPose = new int[]{ YawAngle * 10,  (ActionArray[e_Specific.Duration][ItemNo]), PitchAngle, (ActionArray[e_Specific.Duration][ItemNo])};  // Convert to 0.1 deg and msec
//                result = NativePointSmooth(targetPose);
//                try {
//                    Thread.sleep(ActionArray[e_Specific.Duration][ItemNo]);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//                short pitchEncAfter = (short) (SensorStatus[e_CoreSensor.NeckPitchEnc]);
//                short yawEncAfter = (short) (SensorStatus[e_CoreSensor.NeckYawEnc]);
//                float jointYawAfter = (float) (yawEncAfter) / 4096 * 360;      // Convert encoder count to degree
//                float jointPitchAfter = (float) (pitchEncAfter) / 4096 * 360;
//                float[] JointAfter = {jointYawAfter, jointPitchAfter};
//                do{
//                    if (Math.abs(JointAngle[0] - JointAfter[0]) <= 2 || Math.abs(JointAngle[1] - JointAfter[1]) <= 2) {
//                        ItemNo = ActionArray[e_Specific.NextMotion][ItemNo];
//                        Cycles++;
//                    } else {
//                         Log.i(TAG, "Check Status");
//                         pitchEncAfter = (short) (SensorStatus[e_CoreSensor.NeckPitchEnc]);
//                         yawEncAfter = (short) (SensorStatus[e_CoreSensor.NeckYawEnc]);
//                         jointYawAfter = (float) (yawEncAfter) / 4096 * 360;      // Convert encoder count to degree
//                         jointPitchAfter = (float) (pitchEncAfter) / 4096 * 360;
//                         JointAfter = new float[] {jointYawAfter, jointPitchAfter};
//                        try {
//                            Thread.sleep(ActionArray[e_Specific.Duration][ItemNo]);
//                        } catch (InterruptedException e) {
//                            e.printStackTrace();
//                        }
//                    }
//                }while(Math.abs(JointAngle[0] - JointAfter[0]) <= 2);
//
//                Log.i(TAG,"NextMotion");
//                try {
//                    WaitTime = ActionArray[e_Specific.NextMotionDelay][ItemNo] + bufTime;
//                    Thread.sleep(WaitTime);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }else{
//                targetPose = new int[]{YawAngle * 10, PitchAngle * 10, ActionArray[e_Specific.Duration][ItemNo], ActionArray[e_Specific.Duration][ItemNo]};  // Convert to 0.1 deg and msec
//                result = NativeControlNeckJoint(targetPose);
//                Cycles = 0;
//            }
//            Log.i(TAG, "indexAngle = : " + Integer.toString(indexAngle) + "," + Integer.toString(indexAngle >> 1) + "," + Integer.toString(indexAngle & 1) + "," + Integer.toString(ItemNo));
//            Log.i(TAG, "Abs. Angle [yaw, pitch, time] = : [ " + Integer.toString(targetPose[0]) + " , " + Integer.toString(targetPose[1]) + " , " + Integer.toString(targetPose[2]) + " ]");


            // Wait for a while based on "Duration" and "PauseTime".  Note that "bufTime" msec is appended for guaranteeing the motion is finished
//            do {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//                count++;
//            } while ((count < 10) && (SensorStatus[e_CoreSensor.NeckStatus] & 0x03) > 0);
//            try {
//                WaitTime = ActionArray[e_Specific.PauseTime][ItemNo] + bufTime;
//                Thread.sleep(WaitTime);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

//            Log.i(TAG, "Wait for Duration : " + Integer.toString(ActionArray[e_Specific.Duration][ItemNo]) + " ; PauseTime : " + Integer.toString(ActionArray[e_Specific.PauseTime][ItemNo]));
//
//
//            // Reverse to the original posture
//            if (ActionArray[e_Specific.ReverseTime][ItemNo] > 0) {
//
//                PitchAngle = oldPitchAngle;
//                YawAngle = oldYawAngle;
//                // Send out the command in unit [0.1deg, 0.1deg, msec, msec]
//                targetPose = new int[]{YawAngle * 10, PitchAngle * 10, ActionArray[e_Specific.ReverseTime][ItemNo], ActionArray[e_Specific.ReverseTime][ItemNo]};  // Convert to 0.1 deg and msec
//                result = NativeControlNeckJoint(targetPose);
//
//                // Wait for a while based on "ReverseTime".  Note that "bufTime" msec is appended for guaranteeing the motion is finished,
//                do {
//                    try {
//                        Thread.sleep(100);
//                    } catch (InterruptedException e) {
//                        e.printStackTrace();
//                    }
//                    count++;
//                } while ((count < 10) && (SensorStatus[e_CoreSensor.NeckStatus] & 0x03) > 0);
//                try {
//                    WaitTime = ActionArray[e_Specific.PauseTime][ItemNo] + bufTime;
//                    Thread.sleep(WaitTime);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//
//                Log.i(TAG, "ReverseTime = " + Integer.toString(ActionArray[e_Specific.ReverseTime][ItemNo]));
//                Log.i(TAG, "Reverse to the original posture: [ " + Integer.toString(targetPose[0]) + " , " + Integer.toString(targetPose[1]) + " , " + Integer.toString(targetPose[2]) + " ]");
//            }

//
//            Log.i(TAG, "ActionArray" + Integer.toString(ActionArray[0][ItemNo]) + ", " + Integer.toString(ActionArray[1][ItemNo]) + ", " + Integer.toString(ActionArray[2][ItemNo]) + ", " + Integer.toString(ActionArray[3][ItemNo]) + ", " + Integer.toString(ActionArray[4][ItemNo]) + ", " + Integer.toString(ActionArray[5][ItemNo]) + ", " + Integer.toString(ActionArray[6][ItemNo]) + ", " + Integer.toString(ActionArray[7][ItemNo]) + ", " + Integer.toString(ActionArray[8][ItemNo]) );
//            Log.i(TAG, "NextMotion and Duration: " + Integer.toString(ActionArray[e_Specific.NextMotion][ItemNo]) + ", " + Integer.toString(ActionArray[e_Specific.Duration][ItemNo]) );
//            Log.i(TAG, "e_NextMotionDelay" + Integer.toString(e_Specific.NextMotionDelay) + " ; ItemNo = " + Integer.toString(ItemNo));
//            Log.i(TAG, "ActionArray[e_NextMotionDelay][ItemNo]" + Integer.toString(ActionArray[e_Specific.NextMotionDelay][ItemNo]));
//            Log.i(TAG, "Cycles = " + Integer.toString(Cycles));
//             //Next-step motion, if necessary
//
//            if (ActionArray[e_Specific.NextMotionDelay][ItemNo] >= 0) {
//
//                try {
//                    WaitTime = ActionArray[e_Specific.NextMotionDelay][ItemNo] + bufTime;
//                    Thread.sleep(WaitTime);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//                // Next-step motion //e_NextMotion
//                ItemNo = ActionArray[e_Specific.NextMotion][ItemNo];
//                Cycles++;
//            } else {
//                Cycles = 0;
//            }
//            Log.i(TAG, "e_NextMotionDelay" + Integer.toString(ActionArray[e_Specific.NextMotionDelay][ItemNo]) + " ; e_NextMotion = " + Integer.toString(ActionArray[e_Specific.NextMotion][ItemNo]));
//            Log.i(TAG, "Cycle = " + Integer.toString(Cycles) + " ; ItemNo = " + Integer.toString(ItemNo));
//        } while (Cycles>0 && Cycles<30) ;    // Limit the cycles less than 4
//
//        result = 1;
//        Log.i(TAG, "API -> Finish Specific Action No. " + Integer.toString(ItemNo));
//        return ("Control Neck SpecificAction : " + Integer.toString( result ) );
    }

    /**
     * Drive the robot at a given translational velocity (vx) and rotational velocity (wz)
     * @param vx translational velocity (0.01 m/sec)
     * @param wz (0.01 rad/sec)
     * @return
     */
    public String VelocityControl(int vx, int wz) {

        int cmd[] = {BASECONTROL, vx, wz, 0, 0, 5, 6, 7, 8, 9, 10};
        String result = NativeSetControlParameter(cmd);
        Log.i(TAG, "API -> VelocityControl");
        return result;
    }


    /**
     * remote-control the velocity of the base. Execute this function more than one time will increase the speed.
     * The vx is linear_velocity in (mm/s), wz is angular_velocity (deg/s)
     * max. velocity is limited by 400 mm/sec, and the max. rotational speed is limited by 180 deg/sec
     * @param dir 0: stop, 1: move forward, 2: move backward, 3: turn left, 4: turn right
     * @return true/false
     */
    public String remoteCtrlBase(int dir) {
        int stepVX = 25, stepWZ = 18;
        int maxVX = 400, maxWZ = 180;
        int WHEELBASEBy2_m = 100;       // WHEELBASEBy2_m = 100 mm

        switch (dir) {
            case Forward:     // Speed up
                if (wz != 0)
                    vx = stepVX;
                else if (vx < maxVX)    // max. velocity = 400 mm/sec
                    vx += stepVX;
                wz = 0;
                break;
            case Backward:     // Slow down
                if (wz != 0)
                    vx = -stepVX;
                else if (vx > -maxVX)
                    vx -= stepVX;
                wz = 0;
                break;
            case TurnLeft:     // Turn left
                if (wz < maxWZ)   // max. spin speed = 360 deg/sec
                    wz += stepWZ;   //
                break;
            case TurnRight:     // Turn right
                if (wz > -maxWZ)
                    wz -= stepWZ;
                break;
            default:    // break
                vx = 0;
                wz = 0;
                break;
        }

        // WHEELBASEBy2_m = 100 mm
        int vl = (int) (vx - Math.toRadians(wz) * WHEELBASEBy2_m);  // Limit the max. wheel speed by 400 mm/sec
        int vr = (int) (vx + Math.toRadians(wz) * WHEELBASEBy2_m);
        if (Math.abs(vl) > maxVX)   vl = (int) Math.signum(vl)*maxVX;
        if (Math.abs(vr) > maxVX)   vr = (int) Math.signum(vr)*maxVX;
        int WheelSpd[] = {vl, vr};    // vl = vx - wz * WHEELBASEBy2_m;  vr = vx + wz * WHEELBASEBy2_m;
        int result = NativeWheelSpdCtrl(WheelSpd);

        Log.i(TAG, "API -> Wheel Spd Control : [ " + Integer.toString(vx) + " , " + Integer.toString(wz) + "] or [vl, vr] = ["
                + Integer.toString(WheelSpd[0]) + ", " + Integer.toString(WheelSpd[1]) + " ]");
        return ("Wheel Spd Control : " + Integer.toString( result ) );
    }


    /**
     * Move a relative distance in cartesian coordinate
     * @param iRelX relative distance (m) in x-axis direction
     * @param iRelY relative distance (m) in y-axis direction
     * @param iRelTheta relative rotational angle (deg) in z-axis direction
     * @return true/false
     */
    public String relMoveTo(float iRelX, float iRelY, float iRelTheta){
        int targetPose[] = {(int) (iRelX*1000),(int) (iRelY*1000),(int) (iRelTheta*10)};  // Convert (m, m, deg) to (mm, mm, 0.1 deg)
        int Volume = 1, Index = 0, status = e_rtn.Initial, feedBackStatus;
        int [] SensorStatus;

        Log.i(TAG, "iRelX = " + Integer.toString(targetPose[0]));

        AvoidTargetX = AvoidTargetY = 0;
        if ( NativeRelMoveTo(Volume, Index, targetPose)>=0 ) {  // Successfully send the command to MCU
            feedBackStatus = checkLocomotionStatus();
            while ((feedBackStatus != e_rtn.Pass) && (feedBackStatus != e_rtn.FailTimeout)) {

            }
        }

        Log.i(TAG, "API -> Move a relative distance!");
        return ("Rel. Move To (Cartesian) : " + Integer.toString( status ) );
    }


    /**
     * Move a relative distance in cylindrical coordinate (requested by Darren)
     * @param relRho relative distance (mm)
     * @param relAlpha relative rotational angle (deg) in z-axis direction
     * @return true/false
     */
    public String relMoveToCylindrical(float relRho, float relAlpha, float relBeta){
        float iRelX = (float) (relRho*1000*Math.cos(Math.toRadians(relAlpha)));     // Convert m to mm in x-axis direction
        float iRelY = (float) (relRho*1000*Math.sin(Math.toRadians(relAlpha)));
        float iRelBeta = relBeta*10;                                 // Convert deg to 0.1 deg
        int Volume = 1, Index = 0;

        int targetPose[] = {(int) iRelX, (int) iRelY, (int) iRelBeta};
        int result = NativeRelMoveTo(Volume, Index, targetPose);
        Log.i(TAG, "API -> MoveTo in Cylindrical : [ " + Float.toString(relRho)
                + " , " + Float.toString(relAlpha) + " , " + Float.toString(relBeta) + " ]");
        Log.i(TAG, "API -> MoveTo in Cartesian : [ " + Double.toString(iRelX)
                + " , " + Double.toString(iRelY) + " , " + Float.toString(iRelBeta) + " ]");


        int [] SensorStatus;
        count = 0;
        do {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
            SensorStatus = GetSensorData(0x01);
            Log.d(TAG, Integer.toString(SensorStatus[0x06]));
            count++;
        } while( count < 100 &&  SensorStatus[0x06] == 0x04) ;


        Log.d(TAG, "Finish the relMoveTo (Polar)");
        return ("Rel. Move To (Polar) : " + Integer.toString( result ) );
    }

    /**
     * Move from current position (xA, yA, thetaA) to a target (xB, yB, thetaB) based on the path-finding algorithm.
     * @param xA current location of the robot in x-axis direction
     * @param yA current location of the robot in y-axis direction
     * @param thetaA current heading direction of the robot
     * @param xB target position of the robot in x-axis direction
     * @param yB target position of the robot in y-axis direction
     * @param thetaB target heading direction of the robot
     * @return True/False
     */
/*
    public String goFromAToB(float xA, float yA, float thetaA, float xB, float yB, float thetaB){
        int result = 0, count , bufIndex;
        float gridWidth = 0.05f;   // m
        // Unit : (grid, grid, 0.1deg)
        int initPose[] = {(int) (xA/gridWidth-1), (int) (yA/gridWidth-1), (int) thetaA*10}; //    // int initPose[] = {(int) xA*1000, (int) yA*1000, (int) thetaA*10}; //
        int targetPose[] = {(int) (xB/gridWidth-1), (int) (yB/gridWidth-1), (int) thetaB*10};     // int targetPose[] = {(int) xB*1000, (int) yB*1000, (int) thetaB*10};
        int SensorStatus[];
        int gridPath[];

        // Find a shortest route from A to B
        gridPath = NativeGoFromAToB(initPose, targetPose);  // relative path reference to (xA, yA, thetaA) instead of global position
        int Volume = (gridPath.length)/2;

        for (int Index = 0 ; Index < Volume ; Index++){
            // interpolate points in unit : (mm, mm, 0.1 deg)
            int interPose[] = {(int) (gridPath[Index*2]*gridWidth*1000), (int) (gridPath[Index*2+1]*gridWidth*1000), targetPose[2]};  // interpolate points
            Log.i(TAG, "Point : " + Integer.toString(Index) + "/" + Integer.toString(Volume) +": [" + Integer.toString(interPose[0]) + ", " + Integer.toString(interPose[1]) + ", " + Integer.toString(interPose[2]) + "] ");
            result = NativeRelMoveTo(Volume, Index, interPose);

            count = 0;
            do {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                SensorStatus = GetSensorData(0x01);
                bufIndex = SensorStatus[e_CoreSensor.BaseStatus]>>5 ;
                // Log.d(TAG, "Locomotion Status " + Integer.toString(count) + " : " + Integer.toString(SensorStatus[0x06]) + ", index = " + Integer.toString(bufIndex));
                count++;
            } while ( (count < 100) && bufIndex > 5 ) ;
        }

        count = 0;
        do {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            SensorStatus = GetSensorData(0x01);
            // Log.d(TAG, "Last segments : " + Integer.toString(SensorStatus[0x06]) );
            count++;
        } while ((count < 100) && (SensorStatus[e_CoreSensor.BaseStatus]&0x04)>0 ) ;


        // Since there are more than two points needed to be sent to MCU,
        Log.i(TAG, "API -> Go from an initial pose A to a target pose B!");
        return ("Go from A to B : " + Integer.toString( result ) );
    }
*/
    public String goFromAToB(float xA, float yA, float thetaA, float xB, float yB, float thetaB){
        int Volume, status = e_rtn.Initial;
        float gridWidth = 0.05f;   // m

        //int absInitPose[] = {(int) xA, (int) yA, (int) thetaA};
        // For the A* path-finding, the basic unit used in algorithm is (grid, grid, 0.1deg) instead of (mm, mm, 0.1deg)
        int initPose[] = {(int) (xA/gridWidth-1), (int) (yA/gridWidth-1), (int) thetaA*10}; //    // int initPose[] = {(int) xA*1000, (int) yA*1000, (int) thetaA*10}; //
        int targetPose[] = {(int) (xB/gridWidth-1), (int) (yB/gridWidth-1), (int) thetaB*10};     // int targetPose[] = {(int) xB*1000, (int) yB*1000, (int) thetaB*10};
        int gridPath[];

        while (status == 0 || status == e_rtn.FailObstacle){
            // Find a shortest route from A to B
            AvoidTargetX = AvoidTargetY = 0;
            gridPath = NativeGoFromAToB(initPose, targetPose);  // relative path reference to (xA, yA, thetaA) instead of global position
            Volume = (gridPath.length)/2;

            // Check the buffer size and feed the gridPath to the system
            for (int Index = 0 ; Index < Volume ; Index++){
                // interpolate points in unit : (mm, mm, 0.1 deg)
                int interPose[] = {(int) (gridPath[Index*2]*gridWidth*1000), (int) (gridPath[Index*2+1]*gridWidth*1000), targetPose[2]};  // interpolate points

                status = checkLocomotionBuffer(Index, Volume, interPose);
                if (status == e_rtn.FailObstacle) {
                    initPose[0] = (int) ((AvoidTargetX/1000)/gridWidth-1);  // rel. position (mm) to gridPoint
                    initPose[1] = (int) ((AvoidTargetY/1000)/gridWidth-1);  //
                    // Go back to re-calculate the A* path-finding algorithm
                    break;
                }
            }

            // Wait for the robot finishing the least five points
            if (status == e_rtn.Pass) {
                status = checkLocomotionStatus();
                if (status == e_rtn.FailObstacle) {
                    initPose[0] = (int) ((AvoidTargetX/1000)/gridWidth-1);  // rel. position (mm) to gridPoint
                    initPose[1] = (int) ((AvoidTargetY/1000)/gridWidth-1);  //
                    // Go back to re-calculate the A* path-finding algorithm
                }
            }

        }

        // Since there are more than two points needed to be sent to MCU,
        Log.i(TAG, "API -> Go from an initial pose A to a target pose B!");
        return ("Go from A to B : " + Integer.toString( status ) );
    }



    /**
     *  Move from current position (xA, yA, thetaA) to an estimated target (xB, yB, thetaB) based on the path-finding algorithm. If the estimated target is not available, then replace it with a possible/reasonable target.
     * @param xA current location of the robot in x-axis direction
     * @param yA current location of the robot in y-axis direction
     * @param thetaA current heading direction of the robot
     * @param xB target position of the robot in x-axis direction
     * @param yB target position of the robot in y-axis direction
     * @param thetaB target heading direction of the robot
     * @return True/False
     */
    public String goFromAToBEst(float xA, float yA, float thetaA, float xB, float yB, float thetaB) {
        return "true";
    }


    /**
     * Perform locomotion with multi segments in cartesian coordinate only for self-testing.
     *
     */
    public String relMoveMultiSeg(){
        int targetPose[] = {0, 0, 0};  // Convert (m, m, deg) to (mm, mm, 0.1 deg)
        int index, result = 0;
        int [] SensorStatus;

        int Volume = 8;
        int TempPose[][] = { {1000, 1000, 0}, {2000, 0, 0}, {1000, -1000, 0}, {0, 0, 0},
                {1000, 1000, 0}, {2000, 0, 0}, {1000, -1000, 0}, {0, 0, 0} };


        for (int Index = 0 ; Index < Volume ; Index++) {
            for (int j = 0 ; j < 3 ; j++) {
                targetPose[j] = TempPose[Index][j];
            }
            Log.i(TAG, "Move to : [" + targetPose[0] + ", " + targetPose[1] + ", " + targetPose[2] + "]");
            result = NativeRelMoveTo(Volume, Index, targetPose);

            // Input the targetPose
            count = 0;
            do {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                SensorStatus = GetSensorData(0x01);
                index = SensorStatus[e_CoreSensor.BaseStatus]>>5 ; // The 5th~7th bits indicate the index of the buffer in MCU
                Log.d(TAG, "Locomotion Status " + Integer.toString(count) + " : " + Integer.toString(SensorStatus[e_CoreSensor.BaseStatus]) + ", index = " + Integer.toString(index));
                count++;
            } while ((count < 500) && index > 5) ;  // index > MCUBuffer, where MCUBuffer must be larger than 2 for calculating theta[i+1]

        }


        count = 0;
        do {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            SensorStatus = GetSensorData(0x01);
            Log.d(TAG, "Last segments : " + Integer.toString(SensorStatus[e_CoreSensor.BaseStatus]) );
            count++;
        } while ((count < 500) && (SensorStatus[e_CoreSensor.BaseStatus]&0x04)>0 ) ;

        Log.i(TAG, "API -> Move a relative distance!");
        return ("Rel. Move To (Cartesian) : " + Integer.toString( result ) );
    }



    public String test(int mTimes) {
        Log.i(TAG, "Times : " + Integer.toString(mTimes));
        return "True";
    }


    /**
     * Track the user by rotating head/base without locomotion. The location of the user is calculated by vision system
     * @param cvYawAngle the angle of the user in yaw direction in the image
     * @param cvPitchAngle the angle of the user in pitch direction in the image
     * @return true/false
     */
    public String trackUser(float cvYawAngle, float cvPitchAngle) {     // computer vision
        float Freq = 10;    // 10 Hz
        float yawDiff, pitchDiff, absYawAngle, absPitchAngle;
        float yawDiffMin = 0.5f, yawDiffMax = 3, pitchDiffMin = 0.5f, pitchDiffMax = 3;
        float yawTime = 0.1f, pitchTime = 0.1f;     // 100 msec
        float yawThreshold = 10;   // Drive the wheel to spin around while the yaw angle exceed this threshold (degree)
        int WHEELBASEBy2_m = 100;
        float omega = 50;
        int[] SensorStatus = GetSensorData(0x01);

        // change data type from int32 to short(int16)
        short pitchEncoder = (short)(SensorStatus[e_CoreSensor.NeckPitchEnc]);
        short yawEncoder = (short)(SensorStatus[e_CoreSensor.NeckYawEnc]);
        float jointYawAngle = (float) (yawEncoder) / 4096 * 360;      // Convert encoder count to degree
        float jointPitchAngle = (float) (pitchEncoder) / 4096 * 360;
        if ( Math.abs(cvYawAngle)<1 && Math.abs(cvPitchAngle)<1 ) {
            Log.i(TAG, "The object is inside the bound!");
        } else {
            // Get the current angles in yaw and pitch
            float targetYawAngle = jointYawAngle + cvYawAngle;

            // Determine if the wheel should spin around (yaw direction)
            if (Math.abs(targetYawAngle) > yawThreshold) {
                int vl = (int) (-Math.toRadians(omega * Math.signum(targetYawAngle)) * WHEELBASEBy2_m);  // Limit the max. wheel speed by 400 mm/sec
                int WheelSpd[] = {vl, -vl};    // vl = vx - wz * WHEELBASEBy2_m ;  vr = vx + wz * WHEELBASEBy2_m;
                NativeWheelSpdCtrl(WheelSpd);
            }
            else {
                int WheelSpd[] = {0, 0};
                NativeWheelSpdCtrl(WheelSpd);
            }

            // Calculate the abs angles for both joints
            yawDiff = cvYawAngle/Freq;
            yawDiff = Donut(yawDiff, yawDiffMax, yawDiffMin);   // if the cv value change from 45 to -45 deg
            pitchDiff = cvPitchAngle/Freq;
            pitchDiff = Donut(pitchDiff, pitchDiffMax, pitchDiffMin);
            absYawAngle = Bound((jointYawAngle + yawDiff), 45, -45);
            absPitchAngle = Bound((jointPitchAngle + pitchDiff), 45, -15);

            Log.i("trackUser", "Angle = " + Float.toString(cvYawAngle) + ", " + Float.toString(cvPitchAngle) + "; " + Float.toString(yawDiff) + ", " + Float.toString(pitchDiff) + " ; " + Float.toString(absYawAngle) + ", " + Float.toString(absPitchAngle) + ", " + Float.toString(jointYawAngle) + ", " + Float.toString(jointPitchAngle));
//            Log.i("trackUser", "absYawAngle = " + Float.toString(yawDiff) + ", " + Float.toString(jointYawAngle) + ", "+ Float.toString(absYawAngle));
//            Log.i("trackUser", "absPitchAngle = " + Float.toString(pitchDiff) + ", " + Float.toString(jointPitchAngle) + ", "+ Float.toString(absPitchAngle));
            // Send command to MCU
            ctrlNeckJointTrk(absYawAngle, yawTime, absPitchAngle, pitchTime);   // (degree, sec, degree, sec)

            Log.i("trackUser", "Yaw, Pitch, Time :" + Integer.toString((int) (absYawAngle * 10)) + ", " + Integer.toString((int) (absPitchAngle * 10)) + ", " + Integer.toString((int) (yawTime * 1000)));
//        Log.i("trackUser", "YawEncoder, PitchEncoder" + Integer.toString((int)(yawEncoder*10)) + ", " + Integer.toString((int) (pitchEncoder*10)));
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        return "true";
    }

    private float Bound(float x, float Upper, float Lower) {
        if ( x>Upper) {
            return(Upper);
        } else if ( x<Lower ) {
            return Lower;
        } else {
            return x;
        }
    }

    private float Donut(float x, float Max, float Min) {
        if ( x>Max ) {
            return Max;
        }  else if ( x<Min && x>0) {
            return Min;
        } else if ( x<0 && x>-Min) {
            return -Min;
        }  else if ( x<=-Max ) {
            return -Max;
        } else {    // ( x<=Upper && x>=Lower) || ( x<=-Lower && x>=-Upper) || x==0
            return x;
        }
    }

    /**
     * Follow the user by moving the base instead of rotating its head
     * @param cvRadius the location of the user in Radius direction in image coordinate
     * @param cvYawAngle the location of the user in Yaw direction in image coordinate
     * @param cvPitchAngle the location of the user in Pitch direction in image coordinate
     * @return
     */
    public String followUser(float cvYawAngle, float cvPitchAngle, float cvRadius) {
        // Use "relMoveTo" to perform this but change the target on-the-fly
        float Freq = 5, WHEELBASEBy2_m = 100, velRadius = 0, omega = 0;
        float [] gain = {0.03f, 2}; // {gainVel, gainAngle}
        float [] cvPara = {cvRadius*1000, cvYawAngle};  // {mm degree}
        float [] minPara = {500, 5};  // {Dist(mm) Angle(degree)}
        float [] vel = {velRadius, omega}; // {mm/s, degree/s}
        float pitchDiffMin = 1, pitchDiffMax = 3, yawTime = 0.1f, pitchTime = 0.1f;
        int [] SensorStatus = GetSensorData(0x01);
        // Get pitch angle from encoder and change data type
        short pitchEncoder = (short)(SensorStatus[e_CoreSensor.NeckPitchEnc]);
        short yawEncoder = (short)(SensorStatus[e_CoreSensor.NeckYawEnc]);
        float jointPitchAngle = (float) (pitchEncoder) / 4096 * 360;
        float jointYawAngle = (float) (yawEncoder) / 4096 * 360;      // Convert encoder count to degree
        float pitchDiff = cvPitchAngle/Freq;
        pitchDiff = Donut(pitchDiff, pitchDiffMax, pitchDiffMin);
        float absPitchAngle = Bound((jointPitchAngle + pitchDiff), 45, -15);
        // Check the polarRadius and polarAngle magnitude
        for(int i = 0; i<2; i++){
            if(Math.abs(cvPara[i]) < minPara[i]){
                 vel[i] = 0;
            } else {
                vel[i] = gain[i] * cvPara[i]; //Linear relationship by multiply gain
            }
        }
        // Calculate the right and left wheel speed
        int vl = (int) (vel[0] - Math.toRadians(vel[1]  * WHEELBASEBy2_m));  // Limit the max. wheel speed by 400 mm/sec
        int vr = (int) (vel[0] + Math.toRadians(vel[1]  * WHEELBASEBy2_m));  // Limit the max. wheel speed by 400 mm/sec
        int WheelSpd[] = {vl, vr};
        // Send command to MCU
        NativeWheelSpdCtrl(WheelSpd);
        ctrlNeckJointTrk(jointYawAngle, yawTime, absPitchAngle, pitchTime);   // (degree, sec, degree, sec)
        Log.i("followUser", "pitchAngle = " + Float.toString(pitchDiff) + ", " + Float.toString(jointPitchAngle) + ", " + Float.toString(absPitchAngle) +"," + Float.toString(cvPara[1]) + ", " + Float.toString(vel[1]));
        Log.i("followUser", "vel[1] = " + Float.toString(cvPara[1]) + ", " + Float.toString(vel[1]) + ", " + Float.toString(vl)+ ", " + Float.toString(vr));

        return "true";
    }


    /**
     * Move to a destined position calculated by vision system for auto framing purpose
     * @param x the target position in x-axis direction (m)
     * @param y the target position in y-axis direction (m)
     * @param theta the heading direction of the robot (deg)
     * @return
     */
    public String autoFramingObject(float x, float y, float theta) {
        // Call relMoveto function
        relMoveTo(x, y, theta);
        return "true";
    }

    /**
     * Explore everywhere and move forward 1 meter every time. While moving, the robot will automatically avoid the obstacle at the same time, and the head will not rotate. (yaw angle = 0)
     * @param mode 1: continue the explore process ; 0: finish the explore process
     * @return true: arrive the intermediate target ; false: timeout
     */
    public String exploreAnywhere(int mode) {

        return "true";
    }


    /**
     * Check status of the base while performing locomotion
     * @return
     */
    private int checkLocomotionStatus(){
        int SensorStatus[];
        int status, count = 0;

        do {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            SensorStatus = GetSensorData(0x01);
            if (SensorStatus[e_CoreSensor.AvoidTarget_X] != AvoidTargetX || SensorStatus[e_CoreSensor.AvoidTarget_Y] != AvoidTargetY) {
                status = e_rtn.FailObstacle;    // Stop due to detect an obstacle
                AvoidTargetX = SensorStatus[e_CoreSensor.AvoidTarget_X];
                AvoidTargetY = SensorStatus[e_CoreSensor.AvoidTarget_Y];
                Log.i(TAG, "AvoidTarget_X = " + Integer.toString(AvoidTargetX) + " ; AvoidTarget_Y = " + Integer.toString(AvoidTargetY));
                break;
            } else {
                status = e_rtn.Pass;
            }
            // Log.d(TAG, "Locomotion Status 2 : " + Integer.toString(SensorStatus[e_CoreSensor.BaseStatus]) );
            count++;
            Log.i(TAG, "count : " + Integer.toString(count));
        } while ( (count < 100) && (SensorStatus[e_CoreSensor.BaseStatus]& e_Base.Busy)>0 ) ;
        if (count == 100) {
            status = e_rtn.FailTimeout;
            Log.i(TAG, "status : " + Integer.toString(status));
        }

        return status;
    }


    private int checkLocomotionBuffer(int mIndex, int mVolume, int [] mInterPose) {
        int count , bufIndex, status = e_rtn.Initial;
        int SensorStatus[];

        Log.i(TAG, "Point : " + Integer.toString(mIndex) + "/" + Integer.toString(mVolume) +": [" + Integer.toString(mInterPose[0]) + ", " + Integer.toString(mInterPose[1]) + ", " + Integer.toString(mInterPose[2]) + "] ");
        if ( NativeRelMoveTo(mVolume, mIndex, mInterPose)>=0 ) {
            count = 0;
            do {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                SensorStatus = GetSensorData(0x01);
                bufIndex = SensorStatus[e_CoreSensor.BaseStatus]>>5 ;
                // Check if any obstacle is detected. Note that
                if (SensorStatus[e_CoreSensor.AvoidTarget_X] != AvoidTargetX || SensorStatus[e_CoreSensor.AvoidTarget_Y] != AvoidTargetY) {
                    status = e_rtn.FailObstacle;    // Stop due to detect an obstacle
                    AvoidTargetX = SensorStatus[e_CoreSensor.AvoidTarget_X];
                    AvoidTargetY = SensorStatus[e_CoreSensor.AvoidTarget_Y];
                    Log.i(TAG, "AvoidTarget_X = " + Integer.toString(AvoidTargetX) + " ; AvoidTarget_Y = " + Integer.toString(AvoidTargetY));
                    break;
                } else {    // w/o obstacle detected
                    status = e_rtn.Pass;
                }
                // Log.d(TAG, "Locomotion Status " + Integer.toString(count) + " : " + Integer.toString(SensorStatus[0x06]) + ", index = " + Integer.toString(bufIndex));
                count++;
            } while ( (count < 500) && bufIndex > 5 && status >= e_rtn.Initial) ;
            if (count == 500) {
                status = e_rtn.FailTimeout;
            }
        } else {
            status = e_rtn.FailSendCmd;    //
            Log.e(TAG, "Fail to send interPose : " + Integer.toString(mInterPose[0]) + ", " + Integer.toString(mInterPose[1]) + ", "  + Integer.toString(mInterPose[2]) + ", ");
        }
        return status;
    }



    public interface SensorDataListener {
        void onSenorDataUpdate(int value);
    }

    private static class SensorData {
        final int mMotorData;
        public SensorData(int motordata) {
            mMotorData = motordata;
        }

        public int getMotorData() {
            return mMotorData;
        }
    }

    private static class SensorWatcher implements Runnable {

        private WeakReference<ControlAPI> mWeakRef;

        public SensorWatcher(ControlAPI control) {
            mWeakRef = new WeakReference<ControlAPI>(control);
        }

        @Override
        public void run() {
            ControlAPI tControlApi = mWeakRef.get();
            while(tControlApi != null && tControlApi.mStopmSensorWatcherThread == false) {
                // Start working here
                int motorMsg = 1;
                Message msg = Message.obtain();
                msg.what = 0;
                msg.obj = new SensorData(motorMsg);
                tControlApi.mUpdateHandler.sendMessage(msg);
                try {
                    TimeUnit.MILLISECONDS.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
        }
    }

}
