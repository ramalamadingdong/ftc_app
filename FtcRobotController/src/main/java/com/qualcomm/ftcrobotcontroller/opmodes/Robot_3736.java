package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Rami on 9/11/2015.
 */
public class Robot_3736 extends Robot_3736_Encoder
{
   public Robot_3736 (){ }                                         // Initialise Base Classes

    @Override public void start ()
    {
        super.start ();
        reset_drive_encoders ();
    }
    @Override public void loop ()
    {
        Move (1.0f, 2500);
    }

}
