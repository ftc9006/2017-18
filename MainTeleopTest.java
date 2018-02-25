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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@TeleOp(name="Mainbot: Testing", group="Mainbot")
//@Disabled
public class MainTeleopTest extends OpMode{

    /* Declare OpMode members. */
    HardwareMatthewbot robot       = new HardwareMatthewbot();
    int x = 0;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //start with the ramp down and jewel detector up and aligner in
        robot.colorDrop.setPosition(0.68);
        robot.align.setPosition(0.43);

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
//        double left;
       // robot.colorDrop.setPosition(0.45); //have the jewel detector move slightly out so not in way of flipper

        double Righty, Lefty, sideRight, sideLeft;
        double phaseUp;
        double phaseDown;
        boolean straffeL,straffeR,straffeL2,straffeR2;
        double turnR,turnL;
        double Righty2, Lefty2, sideRight2, sideLeft2;
        double phaseUp2;
        double phaseDown2;

        //set variables to controls on game controller
        Lefty = gamepad1.right_stick_y;
        Righty = gamepad1.left_stick_y;
        sideRight= gamepad1.left_stick_x;
        sideLeft=gamepad1.right_stick_x;
        straffeL = gamepad1.right_bumper;
        straffeR = gamepad1.left_bumper;
        straffeL2 = gamepad2.right_bumper;
        straffeR2 = gamepad2.left_bumper;
        phaseUp2 = gamepad1.right_trigger;
        phaseDown2 = gamepad1.left_trigger;


/*              DRIVING CONTROLS        */

        if(sideRight!=0&&sideLeft<0) {//Straffe while turning left
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft);
            robot.leftRearDrive.setPower(sideLeft * -1);
            robot.rightRearDrive.setPower(sideLeft);
        }
        else if(sideRight!=0&&sideLeft>0)//Straffe while turrning right
        {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft*-1);
            robot.leftRearDrive.setPower(sideLeft);
            robot.rightRearDrive.setPower(sideLeft);
        }
        else if(sideLeft!=0)//normal Straffe
        {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft*-1);
            robot.leftRearDrive.setPower(sideLeft*-1);
            robot.rightRearDrive.setPower(sideLeft);

        }
        else if(Righty<0&&sideRight<0)//rotate left
        {
            robot.leftFrontDrive.setPower(-.5);
            robot.rightFrontDrive.setPower(-.9);
            robot.leftRearDrive.setPower(-.5);
            robot.rightRearDrive.setPower(-.9);
        }
        else if(Righty<0&&sideRight>0) {//rotate right
            robot.leftFrontDrive.setPower(-.9);
            robot.rightFrontDrive.setPower(-.5);
            robot.leftRearDrive.setPower(-.9);
            robot.rightRearDrive.setPower(-.5);
        }
        else if(Righty==0&&sideRight!=0)//spin in place
        {
            robot.leftFrontDrive.setPower(sideRight*-.7);
            robot.rightFrontDrive.setPower(sideRight*.7);
            robot.leftRearDrive.setPower(sideRight*-.7);
            robot.rightRearDrive.setPower(sideRight*.7);
        }
        else if(Righty<0&&sideRight==0)//drive forward/backwards
        {
            robot.leftFrontDrive.setPower(Righty);
            robot.rightFrontDrive.setPower(Righty);
            robot.leftRearDrive.setPower(Righty);
            robot.rightRearDrive.setPower(Righty);

        }
        else if(Righty>0)//drive forward
        {
            robot.leftFrontDrive.setPower(Righty);
            robot.rightFrontDrive.setPower(Righty);
            robot.leftRearDrive.setPower(Righty);
            robot.rightRearDrive.setPower(Righty);

        }
        else if(Lefty!=0&&sideLeft==0)
        {
            robot.leftFrontDrive.setPower(Lefty);
            robot.rightFrontDrive.setPower(Lefty);
            robot.leftRearDrive.setPower(Lefty);
            robot.rightRearDrive.setPower(Lefty);
        }
        else if(straffeL==true||straffeL2==true)//slow straffe left
        {
            robot.leftFrontDrive.setPower(.2);
            robot.rightFrontDrive.setPower(-.2);
            robot.leftRearDrive.setPower(-.2);
            robot.rightRearDrive.setPower(.2);
        }

        else if(straffeR==true||straffeR2==true)//slow straffe right
        {
            robot.leftFrontDrive.setPower(-.2);
            robot.rightFrontDrive.setPower(.2);
            robot.leftRearDrive.setPower(.2);
            robot.rightRearDrive.setPower(-.2);
        }
        else // if not doing the above stop
        {
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);

        }



/*       OTHER CONTROLS         */


        if ((phaseDown2 >0)) {//Left Trigger move the wheels to collect glyphs
            robot.leftStageOne.setPower(1);
            robot.rightStageOne.setPower(1);
        }

        else if ((phaseUp2 >0)) {//Right trigger move wheels to spit out glyphs
            robot.leftStageOne.setPower(-1);
            robot.rightStageOne.setPower(-1);
        }

        else {// if not hitting either trigger stop wheels
            robot.rightStageOne.setPower(0);
            robot.leftStageOne.setPower(0);
        }


        if(gamepad1.dpad_left) {//DPad left puts out alignment rod
            robot.align.setPosition(.43);
        }
        if(gamepad1.dpad_right)//DPad right puts in alignment rod
        {
           robot.align.setPosition(0.90);
        }



        if(gamepad1.dpad_up)
        {
            robot.colorDrop.setPosition(0.05);
        }
        if(gamepad1.dpad_down)
        {
            robot.colorDrop.setPosition(0.68);
        }


        if (gamepad1.a&&robot.digitalTouch.getState() == true){// a puts down the flipper       TRUE is not pressed
            robot.stageTwo.setPower(-.6);
        }
        else if (gamepad1.y){// y lifts flipper all the way
           robot.stageTwo.setPower(.6);
        }
        else
        {
            robot.stageTwo.setPower(0);
        }


        telemetry.addData("cm1", "%.2f cm", robot.rangeSensor1.getDistance(DistanceUnit.CM));//add try catch
     
        telemetry.addData("cm2", "%.2f cm", robot.rangeSensor2.getDistance(DistanceUnit.CM));//add try catch

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}