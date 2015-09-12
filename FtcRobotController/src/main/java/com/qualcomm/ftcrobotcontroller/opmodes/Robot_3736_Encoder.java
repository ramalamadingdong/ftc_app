package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Rami on 9/11/2015.
 */
public class Robot_3736_Encoder extends Robot_3736_Telemetry
{
    public void Move (float speed, int enc) {
        run_using_encoders();
        set_drive_power(speed, speed);

        if (have_drive_encoders_reached(enc, enc)) {
            reset_drive_encoders();                        // Reset Encoders
            set_drive_power(0.0f, 0.0f);                   // Stop Motors
        }
    }
    public void Turn (float speed, int enc)
    {
        run_using_encoders ();
        set_drive_power(speed, -speed);

        if (have_drive_encoders_reached (enc, enc))
        {
            reset_drive_encoders ();                        // Reset Encoders
            set_drive_power (0.0f, 0.0f);                   // Stop Motors
        }
}
}
