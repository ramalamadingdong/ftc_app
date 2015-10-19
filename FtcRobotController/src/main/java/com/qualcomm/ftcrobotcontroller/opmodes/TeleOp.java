

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TeleOp extends OpMode {


	final static double ARM_MIN_RANGE  = 0;
	final static double ARM_MAX_RANGE  = 1;
	final static double BUCKET_MIN_RANGE  = 0;
	final static double BUCKET_MAX_RANGE  = 1;

	double armPosition;


	double armDelta = 0.1;


	double bucketPosition;
// Rami needs to learn how to share

	double bucketDelta = 0.1;

	 DcMotor sweeper;
	 DcMotor lift;
	 DcMotor rightSide;
	 DcMotor leftSide;
	 Servo bucket;
	 Servo arm;

	 /**
	 * Constructor
	 */
	public TeleOp() {

	}

	@Override
	public void init()
    {

		sweeper = hardwareMap.dcMotor.get("MS");
		lift = hardwareMap.dcMotor.get("MLift");
		rightSide = hardwareMap.dcMotor.get("MR");
		leftSide = hardwareMap.dcMotor.get("ML");
		leftSide.setDirection(DcMotor.Direction.REVERSE);

		arm = hardwareMap.servo.get("servo_1");
		bucket = hardwareMap.servo.get("servo_6");

		armPosition = 0.2;
		bucketPosition = 0.2;
	}

	@Override
	public void loop() {

		float y1 = gamepad1.left_stick_y;
		float y2 = gamepad1.right_stick_y;
		float y3 = gamepad2.left_stick_y;
		float y4 = gamepad2.right_stick_y;

		y1 = Range.clip(y1, -1, 1);
		y2 = Range.clip(y2, -1, 1);
		y3 = Range.clip(y3, -1, 1);
		y4 = Range.clip(y4, -1, 1);


		y1 = (float)scaleInput(y1);
		y2 =  (float)scaleInput(y2);
		y3 = (float)scaleInput(y3);

		// write the values to the motors
		sweeper.setPower(y3);
		lift.setPower(y4);

		if (gamepad2.a) {

			armPosition += armDelta;
		}

		if (gamepad2.y) {

			armPosition -= armDelta;
		}


		if (gamepad2.x) {
			bucketPosition += bucketDelta;
		}

		if (gamepad2.b) {
			bucketPosition -= bucketDelta;
		}


        armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
        bucketPosition = Range.clip(bucketPosition, BUCKET_MIN_RANGE, BUCKET_MAX_RANGE);


		arm.setPosition(armPosition);
		bucket.setPosition(bucketPosition);




        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        telemetry.addData("bucket", "bucket:  " + String.format("%.2f", bucketPosition));
	}


	@Override
	public void stop() {

	}

	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
