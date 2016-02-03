package com.asus.control;

import android.util.Log;

import com.asus.control.Constant.e_CoreSensor;
import com.asus.control.Constant.e_Specific;

import java.io.File;
import java.util.Scanner;

/**
 * Created on 2/3/16.
 */
public class SpecificAction {
    private final static String TAG = "SepcificAction";
    private String filePath = "/sdcard/SpecificAction.txt";

    private int rowSize = 9, colSize = 100; // Size of the file
    private int[][] ActionArray = new int[rowSize][colSize];
    private StringBuffer sb = new StringBuffer();

    public int[][] loadFile() {
        // Load file, SpecificAction.txt, and insert into the array
        try {
            int row = 0;
            Scanner inFile = new Scanner(new File(filePath)); // point to the directory
            while (inFile.hasNextLine()) {    // Is there more data to process? return true if there is a line terminator in the input
                int col = 0;
                String Line = inFile.nextLine();
                Scanner inLine = new Scanner(Line);
                while (inLine.hasNextInt()) {
                    ActionArray[row][col] = inLine.nextInt();
                    // sb.append(ActionArray[row][col] + ", ");
                    //Log.i(TAG, "[" + Integer.toString(row) + " , " + Integer.toString(col) + " ] = " + Integer.toString(ActionArray[row][col]));
                    col++;
                }
                sb.append("\n");
                row++;
            }
            // Log.i(TAG, String.valueOf(sb));
            Log.i(TAG, "Successfully load action from the specific action file.");
            inFile.close();
            return ActionArray;
        } catch (Exception e) {
            Log.i(TAG, "Fail to open the file, SpecificAction.txt!");
            e.printStackTrace();
            return null;
        }
    }

//    public void checkStatus(int mTimeDuration){
//        int[] SensorStatus;                     // = GetSensorData(0x01);
//        int count = 0;
//        do {
//            try {
//                Thread.sleep(100);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            count++;
//            SensorStatus = mControlAPI.GetSensorData(0x01);
//            // Log.i(TAG, String.format("Duration = %d, count = %d", (ActionArray[e_Specific.Duration][ItemNo] / 100), count));
//        } while ( count < mTimeDuration && (SensorStatus[e_CoreSensor.NeckStatus]&0x03)>0 );    // In the future, check the status of ( Neck_status = posCtrlBusy[Neck_Yaw] &  posCtrlBusy[Neck_Pitch]<<1 )
//    }


    public void pauseDuration(int mPauseTime) {
        if ( mPauseTime > 5000 ) {   // Assume the pause time is less than 5000 msec.
            mPauseTime = 5000;
        } else if ( mPauseTime <= 0 ) {
            mPauseTime = 0;
        }
        Log.i(TAG, String.format("Finish 1st cycle, and wait for PauseTime = %d", mPauseTime));
        try {
            Thread.sleep(mPauseTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
