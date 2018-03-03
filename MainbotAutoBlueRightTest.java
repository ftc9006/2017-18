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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Locale;

/**
 * This OpMode does the following:
 * Scans the cipher
 * drops sensor to read color
 * read color
 * --knock off jewel--
 * drive to place glyph
 * park
 * if able grab more and stack on the right
 *
 *
 *
 * change parts from pushbot to main bot
 *
 */

@Autonomous(name="Mainbot: Blue Auto Right", group ="Concept")
@Disabled
public class MainbotAutoBlueRightTest extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    Orientation angles;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    OpenGLMatrix lastLocation = null;
    HardwareMatthewbot robot       = new HardwareMatthewbot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static int x = 0,y=0;
    static float z=0,startZ=0,checkZ=0;
    static int position=0;
    static double distance=0;
    double tX , tY , tZ;
    int color =0;//1 red, 2 blue



    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        robot.init(hardwareMap);
        //robot.leftStageTwo.setPosition(.35);robot.rightStageTwo.setPosition(.75);

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//startZ   20
         startZ = AngleUnit.DEGREES.normalize(angles.firstAngle);
        if(startZ<0)
            startZ+=360;


//        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  //      robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        telemetry.addData("Path0",  "Starting at %7d :%7d",
  //              robot.leftDrive.getCurrentPosition(),
    //            robot.rightDrive.getCurrentPosition());
      //  telemetry.update();

        //robot.leftClaw.setPosition(0.7);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.3);

        // get a reference to the color sensor.
//        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
  //      sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AQccD+7/////AAAAGZ+qCVwS3U1qhMXGyIILYuuIvQNwDde781BNyGbcw4QWfN1VlGOdTUaQDJNDnVDmPMVVoYtNxOyAhE6u6X7fFdwAP7EYo5HEXo6VbdJKs5f87V+FultdnK29+6hlnHexuPoV6J5NSkbBCCb/K0LYUKLUiAgaxrxi/cbt3O+k06CXjx0SZv/OLKkmwCQBou2oNm6rNmOTjlb82J9JNWKQtDh6No5mHdJ+QGqdqitGK1/eYxZUrnuwCHdXRDgQ7pFUE6CrI6Hi8qrKWLpdNatPfMmGwSFnNlS7O3E50KOiFL7Z46qwn41ROWWV7k9XOEaT7EbBN6UyewgakNmojMytsasBRSTcgc0W/OO2AXvu1UX0";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

/*
            if(y==0) {
                robot.ramp.setPower(1);
                sleep(2000);
                robot.ramp.setPower(0);
                y=1;
            }

*/
            /**
             *
             * Scan cipher to get LEFT CENTER or RIGHT
             *
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
           if (x==0) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));



                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                     tX = trans.get(0);
                     tY = trans.get(1);
                     tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }

                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                   position=1;
                    vuMark = RelicRecoveryVuMark.UNKNOWN;
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER) {
                   position=2;
                    vuMark = RelicRecoveryVuMark.UNKNOWN;
                }
                else if (vuMark == RelicRecoveryVuMark.LEFT) {
                   position=3;
                    vuMark = RelicRecoveryVuMark.UNKNOWN;
                }
                else
                    position =2;
                /**
                 *
                 * END Scan cipher to get LEFT CENTER or RIGHT
                 *
                 */


                /**
                 *   Drop color sensor  knock off jewel
                 *   write code to lower sensor
                 */


               robot.colorDrop.setPosition(0.05);
                sleep(2000);

                Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                        (int) (robot.sensorColor.green() * SCALE_FACTOR),
                        (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);

                distance = robot.sensorDistance.getDistance(DistanceUnit.CM);
                // send the info back to driver station using telemetry function.
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
             //   telemetry.addData("Alpha", robot.sensorColor.alpha());
             //   telemetry.addData("Red  ", robot.sensorColor.red());
             //   telemetry.addData("Green", robot.sensorColor.green());
             //   telemetry.addData("Blue ", robot.sensorColor.blue());
             //   telemetry.addData("Hue", hsvValues[0]);


                if(robot.sensorColor.red()>robot.sensorColor.blue())
                {
                        color=1;
                        robot.leftFrontDrive.setPower(.25);
                        robot.rightFrontDrive.setPower(.25);
                        robot.leftRearDrive.setPower(.25);
                        robot.rightRearDrive.setPower(.25);

                       sleep(250);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                   sleep(500);
                    robot.colorDrop.setPosition(0.68);
                    sleep(1000);
                }
                else
                {

                    color=2;
                    robot.leftFrontDrive.setPower(.25);
                    robot.rightFrontDrive.setPower(-.25);
                    robot.leftRearDrive.setPower(.25);
                    robot.rightRearDrive.setPower(-.25);



                   sleep(100);
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    robot.colorDrop.setPosition(0.68);
                    sleep(1000);

                    robot.leftFrontDrive.setPower(-.25);
                    robot.rightFrontDrive.setPower(.25);
                    robot.leftRearDrive.setPower(-.25);
                    robot.rightRearDrive.setPower(.25);



                    sleep(100);
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(500);








                }



               // sleep(5000);
                /**
                 *   END Drop color sensor  knock off jewel
                 *   raise color sensor
                 */






               ///telemetry.update();
                /**
                 *   Place Glyph in correct column - redo all code based on mechanism built
                 *
                 */
                if(position==1) {
                    x=1;
                    robot.leftFrontDrive.setPower(.45);
                    robot.rightFrontDrive.setPower(.45);
                    robot.leftRearDrive.setPower(.45);
                    robot.rightRearDrive.setPower(.45);


                    if(color==2)
                    {
                        sleep(1500);
                    }else
                    sleep(925);     // pause for servos to move

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    telemetry.update();
                    sleep(500);

                    do {
                        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        z = AngleUnit.DEGREES.normalize(angles.firstAngle);
                        if(z<0)
                            z+=360;

                        robot.leftFrontDrive.setPower(.25);
                        robot.rightFrontDrive.setPower(-.25);
                        robot.leftRearDrive.setPower(.25);
                        robot.rightRearDrive.setPower(-.25);

                        if(z-startZ<0&&startZ>180) {
                            startZ-=360;
                        }

                    }while(z-startZ>95|z-startZ<85);
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(360);// turn left

                    robot.leftFrontDrive.setPower(.9);
                    robot.rightFrontDrive.setPower(.9);
                    robot.leftRearDrive.setPower(.9);
                    robot.rightRearDrive.setPower(.9);
                    telemetry.update();

                    sleep(275);     // pause for servos to move

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(200);

                    robot.stageTwo.setPower(.6);
                    sleep(400);
                    robot.stageTwo.setPower(0);
                    telemetry.addData("Path", "Complete");
                    telemetry.update();
                    position=0;

                    sleep(1000);

                    robot.leftFrontDrive.setPower(-.25);
                    robot.rightFrontDrive.setPower(-.25);
                    robot.leftRearDrive.setPower(-.25);
                    robot.rightRearDrive.setPower(-.25);


                    sleep(500);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(300);

                    robot.leftFrontDrive.setPower(.45);
                    robot.rightFrontDrive.setPower(.45);
                    robot.leftRearDrive.setPower(.45);
                    robot.rightRearDrive.setPower(.45);
                    telemetry.update();
                    sleep(850);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep (200);

                    robot.leftFrontDrive.setPower(-.45);
                    robot.rightFrontDrive.setPower(-.45);
                    robot.leftRearDrive.setPower(-.45);
                    robot.rightRearDrive.setPower(-.45);
                    telemetry.update();
                    sleep(100);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                }
                if(position==2) {
                    x=1;
                    robot.leftFrontDrive.setPower(.45);
                    robot.rightFrontDrive.setPower(.45);
                    robot.leftRearDrive.setPower(.45);
                    robot.rightRearDrive.setPower(.45);


                    if(color==2)
                    {
                        sleep(1150);
                    }else
                        sleep(625);
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(500);

                    do {
                        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        z = AngleUnit.DEGREES.normalize(angles.firstAngle);
                        if(z<0)
                            z+=360;

                        robot.leftFrontDrive.setPower(.25);
                        robot.rightFrontDrive.setPower(-.25);
                        robot.leftRearDrive.setPower(.25);
                        robot.rightRearDrive.setPower(-.25);

                        if(z-startZ<0&&startZ>180) {
                            startZ-=360;
                        }

                    }while(z-startZ>95|z-startZ<85);//turns left
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(360);// rest

                    robot.leftFrontDrive.setPower(.9);
                    robot.rightFrontDrive.setPower(.9);
                    robot.leftRearDrive.setPower(.9);
                    robot.rightRearDrive.setPower(.9);


                    sleep(275);     // drive forward

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(200);

                    robot.stageTwo.setPower(.6);
                    sleep(400);
                    robot.stageTwo.setPower(0);
                    telemetry.addData("Path", "Complete");
                    telemetry.update();
                    position=0;

                    sleep(1000); // shoot cube

                    robot.leftFrontDrive.setPower(-.25);
                    robot.rightFrontDrive.setPower(-.25);
                    robot.leftRearDrive.setPower(-.25);
                    robot.rightRearDrive.setPower(-.25);


                    sleep(500); // back up to let cube fall

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(300);

                    robot.leftFrontDrive.setPower(.45);
                    robot.rightFrontDrive.setPower(.45);
                    robot.leftRearDrive.setPower(.45);
                    robot.rightRearDrive.setPower(.45);

                    sleep(850);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep (200);

                    robot.leftFrontDrive.setPower(-.45);
                    robot.rightFrontDrive.setPower(-.45);
                    robot.leftRearDrive.setPower(-.45);
                    robot.rightRearDrive.setPower(-.45);
                    telemetry.update();
                    sleep(100);// back away so not touching cube

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                }
                if(position==3) {
                    x=1;
                    robot.leftFrontDrive.setPower(.45);
                    robot.rightFrontDrive.setPower(.45);
                    robot.leftRearDrive.setPower(.45);
                    robot.rightRearDrive.setPower(.45);


                    if(color==2)
                    {
                        sleep(800);
                    }else
                        sleep(450);      // pause for servos to move

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(500);

                    do {
                        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        z = AngleUnit.DEGREES.normalize(angles.firstAngle);
                        if(z<0)
                            z+=360;

                        robot.leftFrontDrive.setPower(.25);
                        robot.rightFrontDrive.setPower(-.25);
                        robot.leftRearDrive.setPower(.25);
                        robot.rightRearDrive.setPower(-.25);

                        if(z-startZ<0&&startZ>180) {
                            startZ-=360;
                        }

                    }while(z-startZ>95|z-startZ<85);
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(360);// turn left

                    robot.leftFrontDrive.setPower(.9);
                    robot.rightFrontDrive.setPower(.9);
                    robot.leftRearDrive.setPower(.9);
                    robot.rightRearDrive.setPower(.9);


                    sleep(275);     // pause for servos to move

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(200);

                    robot.stageTwo.setPower(.6);
                    sleep(400);
                    robot.stageTwo.setPower(0);
                    telemetry.addData("Path", "Complete");
                    telemetry.update();
                    position=0;

                    sleep(1000);

                    robot.leftFrontDrive.setPower(-.25);
                    robot.rightFrontDrive.setPower(-.25);
                    robot.leftRearDrive.setPower(-.25);
                    robot.rightRearDrive.setPower(-.25);


                    sleep(500);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep(300);

                    robot.leftFrontDrive.setPower(.45);
                    robot.rightFrontDrive.setPower(.45);
                    robot.leftRearDrive.setPower(.45);
                    robot.rightRearDrive.setPower(.45);

                    sleep(850);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);

                    sleep (200);

                    robot.leftFrontDrive.setPower(-.45);
                    robot.rightFrontDrive.setPower(-.45);
                    robot.leftRearDrive.setPower(-.45);
                    robot.rightRearDrive.setPower(-.45);
                    telemetry.update();
                    sleep(100);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                }

            }


            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    /**
     * Drive using motor encoders
     *
     *
     * @param speed
     * @param leftInches
     * @param rightInches
     * @param timeoutS
     */

}