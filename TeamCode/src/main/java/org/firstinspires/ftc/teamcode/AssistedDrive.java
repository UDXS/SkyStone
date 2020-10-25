/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "JimBA2 Assisted Drive", group = "Driver")
public class AssistedDrive extends OpMode {

	private ElapsedTime runtime = new ElapsedTime();

	private DcMotor frontDrive, backDrive, leftDrive, rightDrive;

	private DcMotor leftArmBase, rightArmBase;
	private DcMotor leftArmPivot, rightArmPivot;
	private CRServo leftHandGrip, rightHandGrip;

	private float position = 0;


	@Override
	public void init() {

		telemetry.addData("Status", "Mapping Hardware...");
		gamepad1.setJoystickDeadzone(.2f);
		try {
			frontDrive = hardwareMap.get(DcMotor.class, "FrontMotor");
			backDrive = hardwareMap.get(DcMotor.class, "BackMotor");
			leftDrive = hardwareMap.get(DcMotor.class, "LeftMotor");
			rightDrive = hardwareMap.get(DcMotor.class, "RightMotor");

			leftArmBase = hardwareMap.get(DcMotor.class, "LeftArmBase");
			rightArmBase = hardwareMap.get(DcMotor.class, "RightArmBase");

			leftArmPivot = hardwareMap.get(DcMotor.class, "LeftArmPivot");
			rightArmPivot = hardwareMap.get(DcMotor.class, "RightArmPivot");
			leftHandGrip = hardwareMap.get(CRServo.class, "LeftHandGrip");
			rightHandGrip = hardwareMap.get(CRServo.class, "RightHandGrip");

		} catch(Exception e) {
			telemetry.addData("Status", "Mapping Failure.");
			throw e;
		}
		telemetry.addData("Status", "Mapping Completed.");

		// Most robots need the motor on one side to be reversed to drive forward
		// Reverse the motor that runs backwards when connected directly to the battery
		frontDrive.setDirection(DcMotor.Direction.FORWARD);
		backDrive.setDirection(DcMotor.Direction.REVERSE);
		leftDrive.setDirection(DcMotor.Direction.REVERSE);
		rightDrive.setDirection(DcMotor.Direction.FORWARD);

		//leftArmBase.setDirection(DcMotor.Direction.REVERSE);
		//rightArmBase.setDirection(DcMotor.Direction.REVERSE);

		leftArmPivot.setDirection(DcMotor.Direction.REVERSE);
		rightArmPivot.setDirection(DcMotor.Direction.FORWARD);
		// Tell the driver that initialization is complete.
		telemetry.addData("Status", "Initialized");

	}

	float px = 0;
	float py = 0;
	/*
	 * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
	 */
	@Override
	public void init_loop() {

	}

	/*
	 * Code to run ONCE when the driver hits PLAY
	 */
	@Override
	public void start() {
		runtime.reset();
	}

	/*
	 * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
	 */
	@Override
	public void loop() {
		leftArmPivot.setPower(0);
		rightArmPivot.setPower(0);

		if(gamepad1.right_stick_x > .2) {
				frontDrive.setPower(-Math.abs(gamepad1.right_stick_x * 1.2));
				backDrive.setPower(Math.abs(gamepad1.right_stick_x * 1.2));

				leftDrive.setPower(Math.abs(gamepad1.right_stick_x * 1.2));
				rightDrive.setPower(-Math.abs(gamepad1.right_stick_x * 1.2));
			} else if(gamepad1.right_stick_x < -.2){
				frontDrive.setPower(Math.abs(gamepad1.right_stick_x * 1.2));
				backDrive.setPower(-Math.abs(gamepad1.right_stick_x * 1.2));

				leftDrive.setPower(-Math.abs(gamepad1.right_stick_x * 1.2));
				rightDrive.setPower(Math.abs(gamepad1.right_stick_x * 1.2));
			} else {
				frontDrive.setPower(gamepad1.left_stick_x * 1.2);
				backDrive.setPower(gamepad1.left_stick_x * 1.2);

				leftDrive.setPower(gamepad1.left_stick_y * 1.2);
				rightDrive.setPower(gamepad1.left_stick_y * 1.2);
			}

			/*if(gamepad1.left_trigger > .2){
				position += .01;
				if(position > 1) position = 1;
			}
			if(gamepad1.right_trigger > .2){
				position -= .01;
				if(position < 0) position = 0;
			}*/
			//telemetry.addData("right", rightHandGrip.getPosition());
			//telemetry.addData("left", leftHandGrip.getPosition());

			if(gamepad1.right_trigger > .2) {
				leftHandGrip.setDirection(DcMotorSimple.Direction.FORWARD);
				rightHandGrip.setDirection(DcMotorSimple.Direction.REVERSE);
				leftHandGrip.setPower(.3);
				rightHandGrip.setPower(.3);
			} else if(gamepad1.left_trigger > .2) {
				leftHandGrip.setDirection(DcMotorSimple.Direction.REVERSE);
				rightHandGrip.setDirection(DcMotorSimple.Direction.FORWARD);
				leftHandGrip.setPower(.3);
				rightHandGrip.setPower(.3);
			} else {
				leftHandGrip.setPower(0);
				rightHandGrip.setPower(0);
			}

			rightArmBase.setPower(Math.abs(gamepad1.right_stick_y) > 0.3?gamepad1.right_stick_y*-.4:0);
			leftArmBase.setPower(Math.abs(gamepad1.right_stick_y) > 0.3?gamepad1.right_stick_y*.4:0);

			telemetry.update();
	}

	/*
	 * Code to run ONCE after the driver hits STOP
	 */
	@Override
	public void stop() {

	}
}
