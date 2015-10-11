/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TeleOp extends OpMode
{
  ///////////////
  ////SERVOS/////
  ///////////////

  /*
  // position of the claw servo
  double clawPosition;

  // amount to change the claw servo position by
  double clawDelta = 0.01;

  // position of the wrist servo
  double wristPosition;

  // amount to change the wrist servo position by
  double wristDelta = 0.01;

  Servo claw;
  Servo wrist;
*/

  DcMotorController.DeviceMode devMode;
  DcMotorController right_cont;
  DcMotorController left_cont;

  DcMotor LB;
  DcMotor LF;
  DcMotor RB;
  DcMotor RF;

  @Override
  public void init()
  {

    LB = hardwareMap.dcMotor.get("LB");
    LF = hardwareMap.dcMotor.get("LF");
    RB = hardwareMap.dcMotor.get("RB");
    RF = hardwareMap.dcMotor.get("RF");

 /* claw = hardwareMap.servo.get("servo_6"); // channel 6
    wrist = hardwareMap.servo.get("servo_1"); // channel 1*/

    right_cont = hardwareMap.dcMotorController.get("right_cont");
    left_cont = hardwareMap.dcMotorController.get("left_cont");

    devMode = DcMotorController.DeviceMode.WRITE_ONLY;

    LB.setDirection(DcMotor.Direction.REVERSE);
    LF.setDirection(DcMotor.Direction.REVERSE);
  //RB.setDirection(DcMotor.Direction.REVERSE);
  //RF.setDirection(DcMotor.Direction.REVERSE);

    LB.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    LF.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    RB.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    RF.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

    ///////////////
    ////SERVOS/////
    ///////////////
    /*
    wristPosition = 0.6;
    clawPosition = 0.5;
    */
  }

  @Override
  public void loop()
  {
      float y1 = gamepad1.left_stick_y;
      float y2 = gamepad1.right_stick_y;

      y1 = Range.clip(y1, -1, 1);                                                                   // clip the left values so that the values never exceed +/- 1
      y2 = Range.clip(y2, -1, 1);                                                                   // clip the right values so that the values never exceed +/- 1

      LB.setPower(y1);                                                                              // write the values to the motors
      LF.setPower(y1);                                                                              // write the values to the motors
      RB.setPower(y2);                                                                              // write the values to the motors
      RF.setPower(y2);                                                                              // write the values to the motors

      ///////////////
      ////SERVOS/////
      ///////////////
     /* // update the position of the wrist
      if (gamepad1.a) {
        wristPosition -= wristDelta;
      }

      if (gamepad1.y) {
        wristPosition += wristDelta;
      }

      // update the position of the claw
      if (gamepad1.x) {
        clawPosition -= clawDelta;
      }

      if (gamepad1.b) {
        clawPosition += clawDelta;
      }

      // clip the position values so that they never exceed 0..1
      wristPosition = Range.clip(wristPosition, 0, 1);
      clawPosition = Range.clip(clawPosition, 0, 1);

      // write position values to the wrist and claw servo
      wrist.setPosition(wristPosition);
      claw.setPosition(clawPosition);*/

        telemetry.addData("Text", "GOOD LUCK!!");
    }

  // If the device is in either of these two modes, the op mode is allowed to write to the HW.
  private boolean allowedToWrite() {return (devMode == DcMotorController.DeviceMode.WRITE_ONLY);}
}
