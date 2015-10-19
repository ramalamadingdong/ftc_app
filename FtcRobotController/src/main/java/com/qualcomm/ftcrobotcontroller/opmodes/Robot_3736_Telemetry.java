package com.qualcomm.ftcrobotcontroller.opmodes;


/**
 * Created by Rami on 9/11/2015.
 */
public class Robot_3736_Telemetry extends Hardware_with_LinearOpMode
{
    public void update_telemetry ()                                                             // Send telemetry data to the driver station.
    {
        telemetry.addData
                ( "01"
                        , "Left Drive: "
                                + a_left_drive_power ()
                                + ", "
                                + a_left_encoder_count ()
                );
        telemetry.addData
                ( "02"
                        , "Right Drive: "
                                + a_right_drive_power ()
                                + ", "
                                + a_right_encoder_count ()
                );
    /*    telemetry.addData
                ( "03"
                        , "Left Arm: " + a_left_arm_power ()
                );
        telemetry.addData
                ( "04"
                        , "Hand Position: " + a_hand_position ()
                );
                */
    }

}
