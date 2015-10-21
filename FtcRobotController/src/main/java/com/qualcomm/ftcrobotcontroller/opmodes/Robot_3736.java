package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by Rami on 9/11/2015.
 *
 */
public class Robot_3736 extends Robot_3736_Sensors {

    @Override
    public void runOpMode() throws InterruptedException {
        super.start();
        reset_drive_encoders();

        hardwareMap.logDevices();

        cdim = hardwareMap.deviceInterfaceModule.get("dim");                                                    // get a reference to our DeviceInterfaceModule object.

        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);                          // Setting the LED to Output.

        sensorRGB = hardwareMap.colorSensor.get("color");                                                       // get a reference to our ColorSensor object.

        boolean bEnabled = true;                                                                                // bEnabled represents the state of the LED.

        cdim.setDigitalChannelState(LED_CHANNEL, bEnabled);                                                     // turn the LED on in the beginning, just so user will know that the sensor is active.

        waitOneFullHardwareCycle();                                                                             // wait one cycle.

        waitForStart();                                                                                         // wait for the start button to be pressed.

        float hsvValues[] = {0F, 0F, 0F};                                                                       // hsvValues is an array that will hold the hue, saturation, and value information.

        final float values[] = hsvValues;                                                                       // values is a reference to the hsvValues array.

        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        while (opModeIsActive())
        {
            intia();

            cdim.setDigitalChannelState(LED_CHANNEL, bEnabled);                                     // turn on the LED.

            // convert the RGB values to HSV values.
            Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);

            Move(1.0f, 250);
            Turn(1.0f, 250);
        }
    }
}