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
import com.qualcomm.hardware.logitech.LogitechGamepadF310;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

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

    @TeleOp(name="Mainbot: Teleop Tank Brian", group="Pushbot")
    @Disabled
    public class MainTestProgram extends OpMode {

        /* Declare OpMode members. */
        HardwareBrianbot robot = new HardwareBrianbot(); // use the class created to define a Pushbot's hardware
        // could also use HardwarePushbotMatrix class.
        int x = 0;
        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init () {
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
        public void init_loop () {
    }

        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start () {
    }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop () {
        double left;
        double Right, Left, sideRight, sideLeft;
        double phaseUp;
        double phaseDown;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        Right = -gamepad1.left_stick_y;
        Left = -gamepad1.right_stick_y;
        sideRight = gamepad1.right_stick_x;
        sideLeft = gamepad1.left_stick_x;
        phaseUp = gamepad1.right_trigger;
        phaseDown = -gamepad1.left_trigger;


        if (sideLeft != 0)
        {
            robot.leftFrontDrive.setPower(sideLeft * -1);
            robot.rightFrontDrive.setPower(sideLeft);
            robot.leftRearDrive.setPower(sideLeft);
            robot.rightRearDrive.setPower(sideLeft * -1);

        }
        else if (Left > 0 && sideRight > 0)
            {
            robot.leftFrontDrive.setPower(1 - sideRight);
            robot.rightFrontDrive.setPower(Left);
            robot.leftRearDrive.setPower(1 - sideRight);
            robot.rightRearDrive.setPower(Left);
        }
        else if (Left > 0 && sideRight < 0)
        {
            robot.leftFrontDrive.setPower(Left);
            robot.rightFrontDrive.setPower(1 + sideRight);
            robot.leftRearDrive.setPower(Left);
            robot.rightRearDrive.setPower(1 + sideRight);
        }
        else if (Left < 0 && sideRight < 0)
        {
            robot.leftFrontDrive.setPower(1 + sideRight);
            robot.rightFrontDrive.setPower(Left);
            robot.leftRearDrive.setPower(1 + sideRight);
            robot.rightRearDrive.setPower(Left);
        }
        else if (Left < 0 && sideRight > 0)
        {
            robot.leftFrontDrive.setPower(Left);
            robot.rightFrontDrive.setPower(1 - sideRight);
            robot.leftRearDrive.setPower(Left);
            robot.rightRearDrive.setPower(1 - sideRight);
        }
        else if (Left == 0 && sideRight != 0)
        {
            robot.leftFrontDrive.setPower(sideRight);
            robot.rightFrontDrive.setPower(sideRight * -1);
            robot.leftRearDrive.setPower(sideRight);
            robot.rightRearDrive.setPower(sideRight * -1);
        }
        else if (Left != 0)
        {
            robot.leftFrontDrive.setPower(Left);
            robot.rightFrontDrive.setPower(Left);
            robot.leftRearDrive.setPower(Left);
            robot.rightRearDrive.setPower(Left);

        }
        else if (Right != 0)
        {
            robot.leftFrontDrive.setPower(Right);
            robot.rightFrontDrive.setPower(Right);
            robot.leftRearDrive.setPower(Right);
            robot.rightRearDrive.setPower(Right);
        }
        else
        {
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);

        }
        if (phaseUp != 0) {
            robot.leftPhaseOne.setPower(phaseUp);
            robot.rightPhaseOne.setPower(phaseUp);
        } else {
            robot.rightPhaseOne.setPower(0);
            robot.leftPhaseOne.setPower(0);
        }
        if (phaseDown != 0) {
            robot.leftPhaseOne.setPower(phaseDown);
            robot.rightPhaseOne.setPower(phaseDown);
        } else {
            robot.leftPhaseOne.setPower(0);
            robot.rightPhaseOne.setPower(0);
        }

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}