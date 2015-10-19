package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.BoschGyro;
import com.qualcomm.ftcrobotcontroller.MyContext;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Created by bk on 9/21/2015.
 */

public class imuDemo extends OpMode {
    private DeviceInterfaceModule dim;
    private BoschGyro boschGyro;

    private ElapsedTime runTime = new ElapsedTime();
    private String startDate;
    private double delayStopTime;
    public MyContext mMyContext = new MyContext(null);

    public imuDemo() {
    }

    @Override
    public void init()
    {
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());

        // The strings must match names given in Settings->Configure Robot
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        boschGyro = new BoschGyro(dim,0);
        runTime.reset();
    }

    @Override
    public void init_loop()
    {
        if (!boschGyro.initComplete())
        {
            boschGyro.updateState();
            telemetry.addData("1. State",boschGyro.currentState());
        }
        else
        {
            telemetry.addData("2. Heading",boschGyro.heading());
        }
    }

    @Override
    public void loop()
    {
        // Feature to "record" what you want while running
        showDiagnostics();
    }

    @Override
    public void stop() {

    }

    /**
     * Nice little routine to record data to file that can be retrieved via the ADB.
     * adb pull /mnt/shell/emulated/0/Android/data/com.qualcomm.ftcrobotcontroller/files/TraceFile.txt
     * To remove the file since it always appends, issue the following command
     * adb shell rm /mnt/shell/emulated/0/Android/data/com.qualcomm.ftcrobotcontroller/files/TraceFile.txt
     * Need to add something in init to delete the file.
     */
    public void showDiagnostics() {
        String L1 = "L: ";
        String L2 = "R: ";
        String L3 = "T: " + runTime.toString();
        try {
            // Creates a trace file in the primary external storage space of the
            // current application.
            // If the file does not exists, it is created.
            File traceFile = new File(mMyContext.getContext().getExternalFilesDir(null), "TraceFile.txt");
            if (!traceFile.exists())
                traceFile.createNewFile();
            // Adds a line to the trace file
            BufferedWriter writer = new BufferedWriter(new FileWriter(traceFile, true /*append*/));
            writer.write(L1 + "\n" + L2 + "\n" + L3 + "\n\n");
            writer.close();
        } catch (IOException e) {
        }
    }
}
