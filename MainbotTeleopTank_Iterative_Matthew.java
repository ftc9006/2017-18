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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.logitech.LogitechGamepadF310;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

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

@TeleOp(name="Mainbot: Teleop Tank 1P", group="Pushbot")
//@Disabled
public class MainbotTeleopTank_Iterative_Matthew extends OpMode{

    /* Declare OpMode members. */
    HardwareMatthewbot robot       = new HardwareMatthewbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
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
     //   robot.leftBumper.setPosition(.5);
        robot.colorDrop.setPosition(0.35);
        robot.rightStageTwo.setPosition(.99);
        robot.leftStageTwo.setPosition(.12);
      //  robot.rightBumper.setPosition(.7);

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
//        double left;
        robot.colorDrop.setPosition(0.35);

        double Righty, Lefty, sideRight, sideLeft;
        boolean straffeL,straffeR;
        double phaseUp;
        double phaseDown;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        Lefty = gamepad1.left_stick_y;
        Righty = gamepad1.right_stick_y;
        sideRight = gamepad1.right_stick_x;
        sideLeft = gamepad1.left_stick_x;
        phaseUp = gamepad1.right_trigger;
        phaseDown = gamepad1.left_trigger;
        straffeL = gamepad1.right_bumper;
        straffeR = gamepad1.left_bumper;




        if(sideRight<0&&sideLeft<0) {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft);
            robot.leftRearDrive.setPower(sideLeft * -1);
            robot.rightRearDrive.setPower(sideLeft);
        }
        else if(sideRight<0&&sideLeft>0)
        {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft);
            robot.leftRearDrive.setPower(sideLeft);
            robot.rightRearDrive.setPower(sideLeft*-1);
        }
        else if(sideRight>0&&sideLeft>0)
        {
            robot.leftFrontDrive.setPower(sideLeft*-1);
            robot.rightFrontDrive.setPower(sideLeft*-1);
            robot.leftRearDrive.setPower(sideLeft*-1);
            robot.rightRearDrive.setPower(sideLeft);
        }
        else if(sideRight>0&&sideLeft<0)
        {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft*-1);
            robot.leftRearDrive.setPower(sideLeft*-1);
            robot.rightRearDrive.setPower(sideLeft*-1);
        }
        else if(sideLeft<0.5&&sideLeft>-0.5)
        {
            robot.leftFrontDrive.setPower(sideLeft*.5);
            robot.rightFrontDrive.setPower(sideLeft*-.5);
            robot.leftRearDrive.setPower(sideLeft*-.5);
            robot.rightRearDrive.setPower(sideLeft*.5);

        }
        else if(sideLeft!=0)
        {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft*-1);
            robot.leftRearDrive.setPower(sideLeft*-1);
            robot.rightRearDrive.setPower(sideLeft);

        }
        else if(Righty<0&&Righty>-.5&&sideRight<0&&sideRight>-.5)
        {
            robot.leftFrontDrive.setPower(-.25);
            robot.rightFrontDrive.setPower(-.45);
            robot.leftRearDrive.setPower(-.25);
            robot.rightRearDrive.setPower(-.45);
        }
        else if(Righty<0&&sideRight<0)
        {
            robot.leftFrontDrive.setPower(-.5);
            robot.rightFrontDrive.setPower(-.9);
            robot.leftRearDrive.setPower(-.5);
            robot.rightRearDrive.setPower(-.9);
        }
        else if(Righty<0&&Righty>-.5&&sideRight>0&&sideRight<.5) {
            robot.leftFrontDrive.setPower(-.45);
            robot.rightFrontDrive.setPower(-.25);
            robot.leftRearDrive.setPower(-.45);
            robot.rightRearDrive.setPower(-.25);
        }
        else if(Righty<0&&sideRight>0) {
            robot.leftFrontDrive.setPower(-.9);
            robot.rightFrontDrive.setPower(-.5);
            robot.leftRearDrive.setPower(-.9);
            robot.rightRearDrive.setPower(-.5);
        }
        else if(Righty>0&&Righty<.5&&sideRight<0&&sideRight>-.5)
        {
            robot.leftFrontDrive.setPower(.45);
            robot.rightFrontDrive.setPower(.25);
            robot.leftRearDrive.setPower(.45);
            robot.rightRearDrive.setPower(.25);
        }
        else if(Righty>0&&sideRight<0)
        {
            robot.leftFrontDrive.setPower(.9);
            robot.rightFrontDrive.setPower(.5);
            robot.leftRearDrive.setPower(.9);
            robot.rightRearDrive.setPower(.5);
        }
        else if(Righty>0&&Righty<.5&&sideRight>0&&sideRight<.5) {
            robot.leftFrontDrive.setPower(.5);
            robot.rightFrontDrive.setPower(.9);
            robot.leftRearDrive.setPower(.5);
            robot.rightRearDrive.setPower(.9);
        }
        else if(Righty>0&&sideRight>0) {
            robot.leftFrontDrive.setPower(.5);
            robot.rightFrontDrive.setPower(.9);
            robot.leftRearDrive.setPower(.5);
            robot.rightRearDrive.setPower(.9);
        }
        else if(Righty==0&&((sideRight>0&&sideRight<.5)||(sideRight<0&&sideRight>-.5)))
        {
            robot.leftFrontDrive.setPower(sideRight*-.5);
            robot.rightFrontDrive.setPower(sideRight*.5);
            robot.leftRearDrive.setPower(sideRight*-.5);
            robot.rightRearDrive.setPower(sideRight*.5);
        }
        else if(Righty==0&&sideRight!=0)
        {
            robot.leftFrontDrive.setPower(sideRight*-1);
            robot.rightFrontDrive.setPower(sideRight);
            robot.leftRearDrive.setPower(sideRight*-1);
            robot.rightRearDrive.setPower(sideRight);
        }
        else if(Righty<0&&Righty>-.5&&sideRight==0)
        {
            robot.leftFrontDrive.setPower(Righty*.5);
            robot.rightFrontDrive.setPower(Righty*.5);
            robot.leftRearDrive.setPower(Righty*.5);
            robot.rightRearDrive.setPower(Righty*.5);

        }
        else if(Righty<0&&sideRight==0)
        {
            robot.leftFrontDrive.setPower(Righty);
            robot.rightFrontDrive.setPower(Righty);
            robot.leftRearDrive.setPower(Righty);
            robot.rightRearDrive.setPower(Righty);

        }
        else if(Righty>0&&Righty<.5)
        {
            robot.leftFrontDrive.setPower(Righty*.5);
            robot.rightFrontDrive.setPower(Righty*.5);
            robot.leftRearDrive.setPower(Righty*.5);
            robot.rightRearDrive.setPower(Righty*.5);

        }
        else if(Righty>0)
        {
            robot.leftFrontDrive.setPower(Righty);
            robot.rightFrontDrive.setPower(Righty);
            robot.leftRearDrive.setPower(Righty);
            robot.rightRearDrive.setPower(Righty);

        }
        else if(sideLeft==0&&((Lefty>0&&Lefty<.5)||(Lefty<0&&Lefty>-.5)))
        {
            robot.leftFrontDrive.setPower(sideRight*-.5);
            robot.rightFrontDrive.setPower(sideRight*.5);
            robot.leftRearDrive.setPower(sideRight*-.5);
            robot.rightRearDrive.setPower(sideRight*.5);
        }
        else if(Lefty!=0&&sideLeft==0)
        {
            robot.leftFrontDrive.setPower(Lefty);
            robot.rightFrontDrive.setPower(Lefty);
            robot.leftRearDrive.setPower(Lefty);
            robot.rightRearDrive.setPower(Lefty);
        }
        else if(straffeL==true)
        {
            robot.leftFrontDrive.setPower(.2);
            robot.rightFrontDrive.setPower(-.2);
            robot.leftRearDrive.setPower(-.2);
            robot.rightRearDrive.setPower(.2);
        }

        else if(straffeR==true)
        {
            robot.leftFrontDrive.setPower(-.2);
            robot.rightFrontDrive.setPower(.2);
            robot.leftRearDrive.setPower(.2);
            robot.rightRearDrive.setPower(-.2);
        }
        else
        {
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);

        }


        if (phaseDown < 0.5&& phaseDown!=0) {
            robot.leftStageOne.setPower(.5);
            robot.rightStageOne.setPower(.5);
        }
        else if (phaseDown >= 0.5) {
            robot.leftStageOne.setPower(1);
            robot.rightStageOne.setPower(1);
        }
        else if (phaseUp < 0.5&& phaseUp!=0) {
            robot.leftStageOne.setPower(-.5);
            robot.rightStageOne.setPower(-.5);
        }
        else if (phaseUp >= 0.5) {
            robot.leftStageOne.setPower(-1);
            robot.rightStageOne.setPower(-1);
        }
        else {
            robot.rightStageOne.setPower(0);
            robot.leftStageOne.setPower(0);
        }

        if (gamepad1.dpad_up){
            robot.ramp.setPower(1);
        }
        else if(gamepad1.dpad_down){
            robot.ramp.setPower(-1);
        }
        else{
            robot.ramp.setPower(0);
        }
        if (gamepad1.y){
            robot.leftStageTwo.setPosition(1);
            robot.rightStageTwo.setPosition(0.1);
        }
        if(gamepad1.x)
        {
            robot.leftStageTwo.setPosition(.4);
            robot.rightStageTwo.setPosition(.7);
        }
        if (gamepad1.a){
            robot.leftStageTwo.setPosition(0.12);
            robot.rightStageTwo.setPosition(.99);
        }
        if (gamepad1.b){
            robot.colorDrop.setPosition(.35);
        }




/*
        Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                (int) (robot.sensorColor.green() * SCALE_FACTOR),
                (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                hsvValues);


        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", robot.sensorColor.alpha());
        telemetry.addData("Red  ", robot.sensorColor.red());
        telemetry.addData("Green", robot.sensorColor.green());
        telemetry.addData("Blue ", robot.sensorColor.blue());
        */
        telemetry.addData("TriggerR ", phaseUp);
        telemetry.addData("TriggerL ", phaseDown);
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