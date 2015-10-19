package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Rami on 9/11/2015.
 */
public class Robot_3736 extends Robot_3736_Encoder
{
    @Override
    public void runOpMode() throws InterruptedException {
        super.start ();
        reset_drive_encoders ();
    waitForStart();

    while (opModeIsActive())
    {
        Move (1.0f, 2500);
        Turn (1.0f, 2500);
    }
}
}
