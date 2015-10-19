package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class Hardware_with_LinearOpMode extends LinearOpMode {
  private DcMotorController BackDriveCont;

  private DcMotor LB;
  private DcMotor RB;

  final int v_channel_left_drive = 1;
  final int v_channel_right_drive = 2;

  // DcMotor Util1;                                                           // Game Element Motor

  // Servo v_servo_left_hand;

  // Servo v_servo_right_hand;


  @Override
  public void runOpMode() throws InterruptedException { }


  double scale_motor_power (double p_power)
  {
    double l_scale = 0.0f;

    double l_power = Range.clip (p_power, -1, 1);

    double[] l_array =
            { 0.00, 0.05, 0.09, 0.10, 0.12
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
  int a_right_encoder_count (){return RB.getCurrentPosition ();

  }
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