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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    /* Public OpMode members. */
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive  = null;
    public DcMotor  rightRearDrive  = null;
    public DcMotor  leftStageOne   = null; //Cube Collection Mechanism
    public DcMotor  rightStageOne   = null;//Cube Collection Mechanism
    public DcMotor  relicMotor = null;// Scissor Jack
    public Servo    leftBumper = null;
    public Servo    align = null;
    public Servo    rightBumper = null;
    public Servo    relicServo = null;
    public Servo    relicDropperServo = null;
    public Servo    leftStageTwo = null;
    public Servo    rightStageTwo = null;
    public CRServo    ramp = null;
    public Servo    colorDrop = null;
    public BNO055IMU imu=null;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    ColorSensor sensorColor2;
    DistanceSensor sensorDistance2;

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


        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "lfd");
        rightFrontDrive = hwMap.get(DcMotor.class, "rfd");
        leftRearDrive = hwMap.get(DcMotor.class, "lrd");
        rightRearDrive = hwMap.get(DcMotor.class, "rrd");
        leftStageOne = hwMap.get(DcMotor.class, "lp1");
        rightStageOne = hwMap.get(DcMotor.class, "rp1");
      //  leftBumper = hwMap.get(Servo.class, "lb");
       // rightBumper = hwMap.get(Servo.class, "rbumper");
        //relicMotor = hwMap.get(DcMotor.class, "rmotor");
        sensorColor = hwMap.get(ColorSensor.class, "sensor1");
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor1");
        sensorColor2 = hwMap.get(ColorSensor.class, "sensor2");
        sensorDistance2 = hwMap.get(DistanceSensor.class, "sensor2");
    //   relicServo = hwMap.get(Servo.class, "rservo");
  //     relicDropperServo = hwMap.get(Servo.class, "rdservo");
        leftStageTwo = hwMap.get(Servo.class, "lp2");
        rightStageTwo = hwMap.get(Servo.class, "rp2");
        align = hwMap.get(Servo.class, "al");
        colorDrop = hwMap.get(Servo.class, "cd");
        ramp = hwMap.get(CRServo.class, "r");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftStageOne.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightStageOne.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

//        leftStageOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  //      rightStageOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftStageOne.setPower(0);
        rightStageOne.setPower(0);
        ramp.setPower(0);
        leftStageTwo.setPosition(1);
        rightStageTwo.setPosition(0.1);
        colorDrop.setPosition(0.35);
        align.setPosition(.95);
       // relicMotor.setPower(0);
        //relicServo.setPosition(0);
        //relicDropperServo.setPosition(53/255);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //relicMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftStageOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightStageOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
 }

