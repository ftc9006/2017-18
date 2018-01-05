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

@TeleOp(name="Mainbot: Teleop Tank 2P", group="Pushbot")
//@Disabled
public class MainbotTeleopTank_Iterative_Sean extends OpMode{

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
        robot.rightStageTwo.setPosition(.5);
        robot.leftStageTwo.setPosition(.5);
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


        double Righty, Lefty, sideRight, sideLeft;
        double phaseUp;
        double phaseDown;
        boolean straffeL,straffeR,straffeL2,straffeR2;
        double turnR,turnL;
        double Righty2, Lefty2, sideRight2, sideLeft2;
        double phaseUp2;
        double phaseDown2;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        Lefty = gamepad1.left_stick_y;
        Righty = gamepad1.right_stick_y;
        sideRight= gamepad1.right_stick_x;
        sideLeft=gamepad1.left_stick_x;
        phaseUp = gamepad2.right_trigger;
        phaseDown = gamepad2.left_trigger;
        straffeL = gamepad1.right_bumper;
        straffeR = gamepad1.left_bumper;
        straffeL2 = gamepad2.right_bumper;
        straffeR2 = gamepad2.left_bumper;
        turnL = gamepad1.left_trigger;
        turnR=gamepad1.left_trigger;

        Lefty2 = gamepad1.left_stick_y;
        Righty2 = gamepad1.right_stick_y;
        sideRight2= gamepad1.right_stick_x;
        sideLeft2=gamepad1.left_stick_x;
        phaseUp2 = gamepad1.right_trigger;
        phaseDown2 = gamepad1.left_trigger;



        if(sideRight!=0&&sideLeft<0) {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft);
            robot.leftRearDrive.setPower(sideLeft * -1);
            robot.rightRearDrive.setPower(sideLeft);
        }
        else if(sideRight!=0&&sideLeft>0)
        {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft*-1);
            robot.leftRearDrive.setPower(sideLeft);
            robot.rightRearDrive.setPower(sideLeft);
        }
        else if(sideLeft!=0)
        {
            robot.leftFrontDrive.setPower(sideLeft);
            robot.rightFrontDrive.setPower(sideLeft*-1);
            robot.leftRearDrive.setPower(sideLeft*-1);
            robot.rightRearDrive.setPower(sideLeft);

        }
        else if(Righty<0&&sideRight<0)
        {
            robot.leftFrontDrive.setPower(-.5);
            robot.rightFrontDrive.setPower(-.9);
            robot.leftRearDrive.setPower(-.5);
            robot.rightRearDrive.setPower(-.9);
        }
        else if(Righty<0&&sideRight>0) {
            robot.leftFrontDrive.setPower(-.9);
            robot.rightFrontDrive.setPower(-.5);
            robot.leftRearDrive.setPower(-.9);
            robot.rightRearDrive.setPower(-.5);
        }
        else if(Righty==0&&sideRight!=0)
        {
            robot.leftFrontDrive.setPower(sideRight*-1);
            robot.rightFrontDrive.setPower(sideRight);
            robot.leftRearDrive.setPower(sideRight*-1);
            robot.rightRearDrive.setPower(sideRight);
        }
        else if(Righty<0&&sideRight==0)
        {
            robot.leftFrontDrive.setPower(Righty);
            robot.rightFrontDrive.setPower(Righty);
            robot.leftRearDrive.setPower(Righty);
            robot.rightRearDrive.setPower(Righty);

        }
        else if(Righty>0)
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
        else if(straffeL==true||straffeL2==true)
        {
            robot.leftFrontDrive.setPower(.2);
            robot.rightFrontDrive.setPower(-.2);
            robot.leftRearDrive.setPower(-.2);
            robot.rightRearDrive.setPower(.2);
        }

        else if(straffeR==true||straffeR2==true)
        {
            robot.leftFrontDrive.setPower(-.2);
            robot.rightFrontDrive.setPower(.2);
            robot.leftRearDrive.setPower(.2);
            robot.rightRearDrive.setPower(-.2);
        }
        else if (turnL>0) {
            robot.leftFrontDrive.setPower(-.25);
            robot.rightFrontDrive.setPower(-.45);
            robot.leftRearDrive.setPower(-.25);
            robot.rightRearDrive.setPower(-.45);
        }
        else if (turnR>0) {
            robot.leftFrontDrive.setPower(-.45);
            robot.rightFrontDrive.setPower(-.25);
            robot.leftRearDrive.setPower(-.45);
            robot.rightRearDrive.setPower(-.25);
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
        else if (phaseDown > 0.5) {
            robot.leftStageOne.setPower(1);
            robot.rightStageOne.setPower(1);
        }
        else {
            robot.rightStageOne.setPower(0);
            robot.leftStageOne.setPower(0);
        }
        if (phaseUp < 0.5&& phaseUp!=0) {
            robot.leftStageOne.setPower(-.5);
            robot.rightStageOne.setPower(-.5);
        }
        else if (phaseDown > 0.5) {
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
            robot.rightStageTwo.setPosition(.98);
        }

        if (phaseDown != 0) {
            robot.leftStageOne.setPower(1);
            robot.rightStageOne.setPower(1);
        } else {
            robot.rightStageOne.setPower(0);
            robot.leftStageOne.setPower(0);
        }
        if (phaseUp != 0) {
            robot.leftStageOne.setPower(-1);
            robot.rightStageOne.setPower(-1);
        } else {
            robot.leftStageOne.setPower(0);
            robot.rightStageOne.setPower(0);
        }


        //   PLAYER 2

        if (phaseDown2 < 0.5&& phaseDown2!=0) {
            robot.leftStageOne.setPower(.5);
            robot.rightStageOne.setPower(.5);
        }
        else if (phaseDown2 > 0.5) {
            robot.leftStageOne.setPower(1);
            robot.rightStageOne.setPower(1);
        }
        else {
            robot.rightStageOne.setPower(0);
            robot.leftStageOne.setPower(0);
        }
        if (phaseUp2 < 0.5&& phaseUp2!=0) {
            robot.leftStageOne.setPower(-.5);
            robot.rightStageOne.setPower(-.5);
        }
        else if (phaseUp2 > 0.5) {
            robot.leftStageOne.setPower(-1);
            robot.rightStageOne.setPower(-1);
        }
        else {
            robot.rightStageOne.setPower(0);
            robot.leftStageOne.setPower(0);
        }

        if (gamepad2.dpad_up){
            robot.ramp.setPower(1);
        }
        else if(gamepad2.dpad_down){
            robot.ramp.setPower(-1);
        }
        else{
            robot.ramp.setPower(0);
        }
        if (gamepad2.y){
            robot.leftStageTwo.setPosition(1);
            robot.rightStageTwo.setPosition(0.1);
        }
        if(gamepad2.x)
        {
            robot.leftStageTwo.setPosition(.4);
            robot.rightStageTwo.setPosition(.7);
        }
        if (gamepad2.a){
            robot.leftStageTwo.setPosition(0.2);
            robot.rightStageTwo.setPosition(.9);
        }


        /*
        if (gamepad1.right_bumper == true){
        //    robot.leftBumper.setPosition(robot.leftBumper.getPosition()+.05);
            sleep(2000);
        }
        else if (gamepad1.left_bumper == true){
          ///  robot.leftBumper.setPosition(robot.leftBumper.getPosition()-.05);

        }
        else{
            ///robot.leftBumper.setPosition(robot.leftBumper.getPosition());
        }
        */
        /*

         */
   //     if (gamepad2.dpad_down){
     //       robot.relicServo.setPosition(robot.relicServo.getPosition()+.025);
       // }
        //else if(gamepad2.dpad_up){
         //   robot.relicServo.setPosition(robot.relicServo.getPosition()-.025);
       // }
       // else{
        //    robot.relicServo.setPosition(robot.relicServo.getPosition());
        //}

//        if (gamepad2.dpad_left){
 //           robot.relicDropperServo.setPosition(robot.relicDropperServo.getPosition()+.025);
  //      }
    //    else if (gamepad2.dpad_right){
     //       robot.relicDropperServo.setPosition(robot.relicDropperServo.getPosition()-.025);
      //  }
       // else {
        //    robot.relicDropperServo.setPosition(robot.relicDropperServo.getPosition());
        //}
   //     if (gamepad2.left_trigger>5){
 //           robot.relicMotor.setPower(.5);
     //   }
        //else if (gamepad2.left_bumper){
       //     robot.relicMotor.setPower(-.5);
        //}
        //else{
        //    robot.relicMotor.setPower(0);
       // }

        // Send telemetry message to signify robot running;
        //  telemetry.addData("claw",  "Offset = %.2f", clawOffset);
  //      telemetry.addData("sideright",  "%.2f", sideRight);
    //    telemetry.addData("right", "%.2f", Left);
        //    telemetry.addData("sideright",  "%.2f", sideRight);
        //  telemetry.addData("left", "%.2f", sideLeft);
        Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                (int) (robot.sensorColor.green() * SCALE_FACTOR),
                (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        //distance = robot.sensorDistance.getDistance(DistanceUnit.CM);
        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", robot.sensorColor.alpha());
        telemetry.addData("Red  ", robot.sensorColor.red());
        telemetry.addData("Green", robot.sensorColor.green());
        telemetry.addData("Blue ", robot.sensorColor.blue());
        telemetry.addData("Righty ", Righty);
        telemetry.addData("sideRight ", sideRight);
      //  telemetry.addData("right bumper: ", robot.rightBumper.getPosition());
   //     telemetry.addData("left bumper: ", robot.leftBumper.getPosition());
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