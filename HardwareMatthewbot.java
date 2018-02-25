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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.DCTree;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See MainbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareMatthewbot
{
    //   Motors
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive  = null;
    public DcMotor  rightRearDrive  = null;
    public DcMotor  leftStageOne   = null; //Cube Collection Mechanism
    public DcMotor  rightStageOne   = null;//Cube Collection Mechanism
    public DcMotor  relicMotor = null;// Scissor Jack
    public DcMotor  stageTwo = null;
    public DcMotor BadMeme = null; // meme




    //Servos
    public Servo    leftBumper = null;
    public Servo    align = null;
    public Servo    rightBumper = null;
    public Servo    relicServo = null;
    public Servo    relicDropperServo = null;
    public CRServo  ramp = null;
    public Servo    colorDrop = null;

    //Sensors
    public BNO055IMU imu=null; //for gyro
    ColorSensor sensorColor; // for rev color proximity sensor
    DistanceSensor sensorDistance; // for rev color proximity sensor
 //   ColorSensor sensorColor2;
   // DistanceSensor sensorDistance2;
    ModernRoboticsI2cRangeSensor rangeSensor1;// for ultrasonic sensor
    ModernRoboticsI2cRangeSensor rangeSensor2;// for ultrasonic sensor
    DigitalChannel digitalTouch; // touch sensor

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMatthewbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;



        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "lfd");
        rightFrontDrive = hwMap.get(DcMotor.class, "rfd");
        leftRearDrive = hwMap.get(DcMotor.class, "lrd");
        rightRearDrive = hwMap.get(DcMotor.class, "rrd");
        leftStageOne = hwMap.get(DcMotor.class, "lp1");
        rightStageOne = hwMap.get(DcMotor.class, "rp1");
        stageTwo = hwMap.get(DcMotor.class,"s2");
       // BadMeme = hwMap.get(DcMotor.class, "u gay");

         // define and initialize servos
        align = hwMap.get(Servo.class, "al");//2
        colorDrop = hwMap.get(Servo.class, "cd");//3
        ramp = hwMap.get(CRServo.class, "r");//0


        //define and initialize sensors
        sensorColor = hwMap.get(ColorSensor.class, "sensor1"); //these are for the jewel detector
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor1"); //these are for the jewel detector

        imu = hwMap.get(BNO055IMU.class, "imu"); //these are for the gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();//these are for the gyro
        imu.initialize(parameters);//these are for the gyro

        rangeSensor1 = hwMap.get(ModernRoboticsI2cRangeSensor.class, "US1");//these are for ultrasonic range sensors
        rangeSensor2 = hwMap.get(ModernRoboticsI2cRangeSensor.class, "US2");//these are for ultrasonic range sensors

        digitalTouch = hwMap.get(DigitalChannel.class, "touch");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        //  set direction motors spin
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftStageOne.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightStageOne.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        stageTwo.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftStageOne.setPower(0);
        rightStageOne.setPower(0);
        stageTwo.setPower(0);


        colorDrop.setPosition(0.75); //put the jewel arm up
        align.setPosition(.95); // put the glyph aligner in


        //set motors to run without encoders
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftStageOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightStageOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stageTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
 }

