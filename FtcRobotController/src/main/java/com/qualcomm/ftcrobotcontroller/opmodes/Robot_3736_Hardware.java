package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Rami on 9/11/2015.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Robot_3736_Hardware extends OpMode
{
    private DcMotorController BackDriveCont;

    private DcMotor LB;
    private DcMotor RB;

    DcMotor sweeper;
    DcMotor lift;
    DcMotor rightSide;
    DcMotor leftSide;
    Servo bucket;
    Servo arm;

    final int v_channel_left_drive = 1;
    final int v_channel_right_drive = 2;

    // DcMotor Util1;                                                           // Game Element Motor

    // Servo v_servo_left_hand;

    // Servo v_servo_right_hand;

    public Robot_3736_Hardware() {}                                             // Initialization

    @Override public void init ()
    {
        sweeper = hardwareMap.dcMotor.get("MS");
        lift = hardwareMap.dcMotor.get("MLift");
        rightSide = hardwareMap.dcMotor.get("MR");
        leftSide = hardwareMap.dcMotor.get("ML");
        leftSide.setDirection(DcMotor.Direction.REVERSE);

        arm = hardwareMap.servo.get("servo_1");
        bucket = hardwareMap.servo.get("servo_6");

        BackDriveCont = hardwareMap.dcMotorController.get("BackDriveCont");

        LB = hardwareMap.dcMotor.get ("LB");
        RB = hardwareMap.dcMotor.get ("right_drive");

        RB.setDirection (DcMotor.Direction.REVERSE);

    /*    Util1 = hardwareMap.dcMotor.get ("Util1");

        double l_hand_position = 0.5;

        v_servo_left_hand = hardwareMap.servo.get ("left_hand");
        v_servo_left_hand.setPosition (l_hand_position);

        v_servo_right_hand = hardwareMap.servo.get ("right_hand");
        v_servo_right_hand.setPosition (l_hand_position); */
    }

    @Override public void start (){}                                            // Keep Compiler Happy
    @Override public void loop (){}                                             // Keep Compiler Happy
    @Override public void stop (){}                                             // Keep Compiler Happy

    double scale_motor_power (double p_power)
    {
        double l_scale = 0.0f;

        double l_power = Range.clip (p_power, -1, 1);

        double[] l_array =
                {
                          0.00, 0.05, 0.09, 0.10, 0.12
                        , 0.15, 0.18, 0.24, 0.30, 0.36
                        , 0.43, 0.50, 0.60, 0.72, 0.85
                        , 1.00, 1.00
                };
        int l_index = (int) (l_power * 16.0);
         if (l_index < 0)
        {
            l_index = -l_index;
        }
         else if (l_index > 16)
        {
            l_index = 16;
        }

         if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
         else
        {
            l_scale = l_array[l_index];
        }
        return l_scale;
    }

    double a_left_drive_power () {return LB.getPower ();}

    double a_right_drive_power (){return RB.getPower ();}

    public void run_using_encoders ()
    {
        DcMotorController.RunMode l_mode = BackDriveCont.getMotorChannelMode  ( v_channel_left_drive );
        if (l_mode == DcMotorController.RunMode.RESET_ENCODERS)
        {
            BackDriveCont.setMotorChannelMode ( v_channel_left_drive, DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        l_mode = BackDriveCont.getMotorChannelMode ( v_channel_right_drive);
        if (l_mode == DcMotorController.RunMode.RESET_ENCODERS)
        {
            BackDriveCont.setMotorChannelMode ( v_channel_right_drive, DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }
    void set_drive_power (double p_left_power, double p_right_power)
    {
        LB.setPower (p_left_power);
        RB.setPower (p_right_power);
    }

    public void reset_drive_encoders ()

    {
        BackDriveCont.setMotorChannelMode( v_channel_left_drive, DcMotorController.RunMode.RESET_ENCODERS);
        BackDriveCont.setMotorChannelMode( v_channel_right_drive, DcMotorController.RunMode.RESET_ENCODERS);
    }

    int a_left_encoder_count (){return LB.getCurrentPosition ();}
    int a_right_encoder_count (){return RB.getCurrentPosition ();}

    boolean have_drive_encoders_reached ( double p_left_count , double p_right_count)
    {
        boolean l_status = false;                                                     // Assume failure.

        //
        // Have the encoders reached the specified values?
        //
        // TODO Implement stall code using these variables.
        //
        if ((Math.abs (LB.getCurrentPosition ()) > p_left_count) && (Math.abs (RB.getCurrentPosition ()) > p_right_count))
        {
            l_status = true;
        }

        return l_status;
    }

    boolean have_drive_encoders_reset ()
    {
        boolean l_status = false;                                                                      // Assume failure.

        if ((a_left_encoder_count () == 0) && (a_right_encoder_count () == 0))                         // Have the encoders reached zero?
        {
            l_status = true;
        }
        return l_status;
    }
}