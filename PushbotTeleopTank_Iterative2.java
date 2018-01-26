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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
//@Disabled
public class PushbotTeleopTank_Iterative2 extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot2 robot       = new HardwarePushbot2(); // use the class created to define a Pushbot's hardware



                                                           // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left,right,speed=0;
        int target=0;


        // set the digital channel to input.

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        if(left==1||left==-1)
        robot.leftDrive.setPower(left);
        else if(left>0&&left<1)
            robot.leftDrive.setPower(.2);
        else if(left<0&&left>-1)
            robot.leftDrive.setPower(-.2);
        else
            robot.leftDrive.setPower(0);

        if(right==1||right==-1)
            robot.rightDrive.setPower(-right);
        else if(left>0&&left<1)
            robot.rightDrive.setPower(.2);
        else if(right<0&&right>-1)
            robot.rightDrive.setPower(-.2);
        else
            robot.rightDrive.setPower(0);


        robot.rightDrive.setPower(right);




        /*

        if (gamepad1.x)
    target = 1000;
  else if (gamepad1.x)
    target = 0;

  // how close are we to final position?
  if (Math.abs(target - rightFlipMotor.getTargetPosition()) < 100)
    speed = 0.1;
  else
    speed = 0.8;

 rightFlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Can't hurt to call this repeatedly
 rightFlipMotor.setPower(speed);




         */
        /*
        if (gamepad1.left_bumper == true)
        {

            target = 240;
            robot.leftArm.setTargetPosition(target);
            telemetry.addData("Flipper", robot.leftArm.getCurrentPosition());
            telemetry.update();
            if (Math.abs(target - robot.leftArm.getTargetPosition()) < 100)
                speed = 0.1;
            else
                speed = 0.8;

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setPower(speed);
        }
        else if (gamepad1.right_bumper == true) {
            target = 0;
            robot.leftArm.setTargetPosition(target);
            telemetry.addData("Flipper", robot.leftArm.getCurrentPosition());
            telemetry.update();
            if (Math.abs(target - robot.leftArm.getTargetPosition()) < 100)
                speed = -0.1;
            else
                speed = -0.8;

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setPower(speed);
        }
        else
        {
            robot.leftArm.setPower(0);
        }
        */
        if (gamepad1.left_bumper == true)
        {
            robot.leftArm.setPower(-.5);
            if (robot.digitalTouch.getState() == false)
                robot.leftArm.setPower(0);
        }
        else if (gamepad1.right_bumper == true) {

            robot.leftArm.setPower(.5);
            if (robot.digitalTouch2.getState() == false)
                robot.leftArm.setPower(0);
        }
        else
        {
            robot.leftArm.setPower(0);
        }




        if (gamepad1.left_trigger>0) {
            robot.leftWheel.setPower(-1.0);
        //    robot.rightWheel.setPower(-1.0);
        }
        else if (gamepad1.right_trigger>0) {
            robot.leftWheel.setPower(1.0);
          //  robot.rightWheel.setPower(1.0);
        }


        if(gamepad1.b)
            robot.leftWheel.setPower(0);
            //robot.rightWheel.setPower(0);



        if(gamepad1.a)
            robot.align.setPosition(.3);
        if(gamepad1.y)
            robot.align.setPosition(.7);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("Flipper", robot.leftArm.getCurrentPosition());
        telemetry.update();
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
