package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;


/**
 * Created by Rami on 10/18/2015.
 */
public class Robot_3736_Sensors extends Robot_3736_Encoder
{
    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;

}
