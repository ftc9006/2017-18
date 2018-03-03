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
 */

@Autonomous(name="Mainbot: Red Left 2", group ="a")
//@Disabled
public class MainbotAutoRedLeftTest2 extends LinearOpMode {

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
    final double SCALE_FACTOR = 255;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    static int x = 0,y=0;
    static float z=0,startZ=0,checkZ=0;
    static int position=0;
    static double distance=0;
    double tX , tY , tZ;
    int color =0;//1 red, 2 blue
    static int shift1=100;
    static int shift2=500;
    static int shift3=700;
static int State= 0;


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        robot.init(hardwareMap);

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // record the starting orientation of the robot
         startZ = AngleUnit.DEGREES.normalize(angles.firstAngle);
        if(startZ<0)
            startZ+=360;








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


            /**
             *
             * Scan cipher to get LEFT CENTER or RIGHT
             *
             */

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.UNKNOWN&&runtime.time()<250) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));
                telemetry.update();


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
            }
            else
            {
                if(State==0) {
                    if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        telemetry.update();
                        position = 1;
                        telemetry.addData("position", position);
                        vuMark = RelicRecoveryVuMark.UNKNOWN;

                    } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        telemetry.update();
                        position = 2;
                        telemetry.addData("position", position);
                        vuMark = RelicRecoveryVuMark.UNKNOWN;

                    } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        telemetry.update();
                        position = 3;
                        telemetry.addData("position", position);
                        vuMark = RelicRecoveryVuMark.UNKNOWN;

                    } else {
                        sleep(500);
                        position = 2;
                    }
                }

               /**
                *
                * END Scan cipher to get LEFT CENTER or RIGHT
                *
                */


               /**
                *   Drop color sensor  knock off jewel
                */
                if (x==0) {
                    if(State==0) hitJewel();


                    /**
                     * Drive to get lined up and shoot
                     */
                    driveToScore();

                    /**
                     * Go get more glyphs
                     */
                    //extraGlyph();

                }
           }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }



    public void hitJewel()
    {
        robot.colorDrop.setPosition(0.05);
        sleep(2000);

        Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                (int) (robot.sensorColor.green() * SCALE_FACTOR),
                (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        if(robot.sensorColor.red()>robot.sensorColor.blue())
        {
            color=1;
            robot.leftFrontDrive.setPower(-.25);
            robot.rightFrontDrive.setPower(-.25);
            robot.leftRearDrive.setPower(-.25);
            robot.rightRearDrive.setPower(-.25);

            sleep(250);

            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.colorDrop.setPosition(0.68);
            sleep(600);
        }
        else
        {

            color=2;
            robot.leftFrontDrive.setPower(-.25);
            robot.rightFrontDrive.setPower(.25);
            robot.leftRearDrive.setPower(-.25);
            robot.rightRearDrive.setPower(.25);



            sleep(100);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.colorDrop.setPosition(0.68);
            sleep(500);

            robot.leftFrontDrive.setPower(.25);
            robot.rightFrontDrive.setPower(-.25);
            robot.leftRearDrive.setPower(.25);
            robot.rightRearDrive.setPower(-.25);

            sleep(100);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            sleep(600);

        }
    }

    public void driveToScore() {
        if (State == 0) {

            //drive until off platform
            robot.leftFrontDrive.setPower(-.45);
            robot.rightFrontDrive.setPower(-.45);
            robot.leftRearDrive.setPower(-.45);
            robot.rightRearDrive.setPower(-.45);
            if (color == 1)
                sleep(200);
            else if (color == 2)
                sleep(600);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            sleep(100);
            telemetry.addData("State",State);
            telemetry.update();
            State = 1;
        }
        //rotate to face wall
        if (State == 1) {
            telemetry.addData("State",State);
            telemetry.update();
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            z = AngleUnit.DEGREES.normalize(angles.firstAngle);
            if (z < 0)
                z += 360;

            robot.leftFrontDrive.setPower(.25);
            robot.rightFrontDrive.setPower(-.25);
            robot.leftRearDrive.setPower(.25);
            robot.rightRearDrive.setPower(-.25);

            if (z - startZ > 75 || z - startZ < 71) {
                State = 2;
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.leftRearDrive.setPower(0);
                robot.rightRearDrive.setPower(0);
                sleep(100);
            }

        }


        //drive to wall
        if (State == 2) {
            telemetry.addData("State",State);
            telemetry.update();
            try {

                robot.leftFrontDrive.setPower(.25);
                robot.rightFrontDrive.setPower(.25);
                robot.leftRearDrive.setPower(.25);
                robot.rightRearDrive.setPower(.25);


              //  telemetry.addData("cm1", "%.2f cm", robot.rangeSensor1.getDistance(DistanceUnit.CM));//add try catch

              //  telemetry.addData("cm2", "%.2f cm", robot.rangeSensor2.getDistance(DistanceUnit.CM));//add try catch

                telemetry.update();

            }
         catch(Exception e){
            telemetry.addData("err", "distance2");//add try catch

            telemetry.update();

        }
        if(robot.rangeSensor2.getDistance(DistanceUnit.CM) > 17) {
            State = 3;
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            sleep(100);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(-.2);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(-.2);
            sleep(50);
        }
    }

        //strafe until lined up
        if(State==3){
            telemetry.addData("State",State);
            telemetry.update();
            robot.leftFrontDrive.setPower(.2);
            robot.rightFrontDrive.setPower(-.2);
            robot.leftRearDrive.setPower(-.25);
            robot.rightRearDrive.setPower(.25);
//            telemetry.addData("cm1", "%.2f cm", robot.rangeSensor1.getDistance(DistanceUnit.CM));//add try catch

  //          telemetry.addData("cm2", "%.2f cm", robot.rangeSensor2.getDistance(DistanceUnit.CM));//add try catch

            telemetry.update();
            if(robot.rangeSensor2.getDistance(DistanceUnit.CM) > 12)
            {
                State=4;
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.leftRearDrive.setPower(0);
                robot.rightRearDrive.setPower(0);
                sleep(100);

                robot.leftFrontDrive.setPower(.2);
                robot.rightFrontDrive.setPower(-.2);
                robot.leftRearDrive.setPower(-.2);
                robot.rightRearDrive.setPower(.2);
                sleep(150);
            }
        }


        if(State==4)
        {
            telemetry.addData("State",State);
            telemetry.update();
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(.2);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(.2);
            if(robot.rangeSensor1.getDistance(DistanceUnit.CM) > 12)
            {
                State=5;
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.leftRearDrive.setPower(0);
                robot.rightRearDrive.setPower(0);
                sleep(100);

                robot.leftFrontDrive.setPower(.25);
                robot.rightFrontDrive.setPower(.25);
                robot.leftRearDrive.setPower(.25);
                robot.rightRearDrive.setPower(.25);
                sleep(100);

                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.leftRearDrive.setPower(0);
                robot.rightRearDrive.setPower(0);
                sleep(100);
            }
        }



        if (position == 1)     //Right Column
        {
             State=11;
        } else if (position == 2)     // Center Column
        {

            if(State==5) {
                telemetry.addData("State",State);
                telemetry.update();

                robot.leftFrontDrive.setPower(.35);
                robot.rightFrontDrive.setPower(-.35);
                robot.leftRearDrive.setPower(-.35);
                robot.rightRearDrive.setPower(.35);
                sleep(500);
                State = 6;
            }

            if(State==6) {
                telemetry.addData("State",State);
                telemetry.update();
                robot.leftFrontDrive.setPower(.2);
                robot.rightFrontDrive.setPower(-.2);
                robot.leftRearDrive.setPower(-.25);
                robot.rightRearDrive.setPower(.25);
//                telemetry.addData("cm1", "%.2f cm", robot.rangeSensor1.getDistance(DistanceUnit.CM));//add try catch

  //              telemetry.addData("cm2", "%.2f cm", robot.rangeSensor2.getDistance(DistanceUnit.CM));//add try catch

                telemetry.update();
                if(robot.rangeSensor2.getDistance(DistanceUnit.CM) > 14)
                {
                    State=7;
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(100);

                    robot.leftFrontDrive.setPower(.2);
                    robot.rightFrontDrive.setPower(-.2);
                    robot.leftRearDrive.setPower(-.2);
                    robot.rightRearDrive.setPower(.2);
                    sleep(150);
                }
            }

          if(State==7){
              telemetry.addData("State",State);
              telemetry.update();
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(.2);
                robot.leftRearDrive.setPower(0);
                robot.rightRearDrive.setPower(.2);
                if(robot.rangeSensor1.getDistance(DistanceUnit.CM) > 17)
                {
                    State=11;
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(100);
                    robot.leftFrontDrive.setPower(.25);
                    robot.rightFrontDrive.setPower(.25);
                    robot.leftRearDrive.setPower(.25);
                    robot.rightRearDrive.setPower(.25);
                    sleep(100);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(100);
                }
            }

        }


        if (position==3)     // Left Column
        {
            if(State==5) {
                telemetry.addData("State",State);
                telemetry.update();
                robot.leftFrontDrive.setPower(.35);
                robot.rightFrontDrive.setPower(-.35);
                robot.leftRearDrive.setPower(-.35);
                robot.rightRearDrive.setPower(.35);
                sleep(500);
                State = 6;
            }

            if(State==6) {
                telemetry.addData("State",State);
                telemetry.update();
                robot.leftFrontDrive.setPower(.2);
                robot.rightFrontDrive.setPower(-.2);
                robot.leftRearDrive.setPower(-.25);
                robot.rightRearDrive.setPower(.25);
//                telemetry.addData("cm1", "%.2f cm", robot.rangeSensor1.getDistance(DistanceUnit.CM));//add try catch

  //              telemetry.addData("cm2", "%.2f cm", robot.rangeSensor2.getDistance(DistanceUnit.CM));//add try catch

                telemetry.update();
                if(robot.rangeSensor2.getDistance(DistanceUnit.CM)>14)
                {
                    State=7;
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(100);

                    robot.leftFrontDrive.setPower(.2);
                    robot.rightFrontDrive.setPower(-.2);
                    robot.leftRearDrive.setPower(-.25);
                    robot.rightRearDrive.setPower(.25);
                    sleep(150);
                }
            }

            if(State==7)
            {
                telemetry.addData("State",State);
                telemetry.update();
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(.2);
                robot.leftRearDrive.setPower(0);
                robot.rightRearDrive.setPower(.2);
                if(robot.rangeSensor1.getDistance(DistanceUnit.CM)>14)
                {
                    State=8;
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(100);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(-.25);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(-.25);
                    sleep(50);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(100);

                    robot.leftFrontDrive.setPower(.35);
                    robot.rightFrontDrive.setPower(-.35);
                    robot.leftRearDrive.setPower(-.35);
                    robot.rightRearDrive.setPower(.35);
                    sleep (500);
                }
            }


            if(State==8) {
                telemetry.addData("State",State);
                telemetry.update();
                robot.leftFrontDrive.setPower(.2);
                robot.rightFrontDrive.setPower(-.2);
                robot.leftRearDrive.setPower(-.25);
                robot.rightRearDrive.setPower(.25);
    //            telemetry.addData("cm1", "%.2f cm", robot.rangeSensor1.getDistance(DistanceUnit.CM));//add try catch

//                telemetry.addData("cm2", "%.2f cm", robot.rangeSensor2.getDistance(DistanceUnit.CM));//add try catch

                telemetry.update();
                if(robot.rangeSensor2.getDistance(DistanceUnit.CM)>17)
                {
                    State=9;
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(500);
                }
            }

            if(State==9)
            {
                telemetry.addData("State",State);
                telemetry.update();
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(.2);
                robot.leftRearDrive.setPower(0);
                robot.rightRearDrive.setPower(.2);
                if(robot.rangeSensor1.getDistance(DistanceUnit.CM)>17)
                {
                    State=11;
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(100);

                    robot.leftFrontDrive.setPower(.15);
                    robot.rightFrontDrive.setPower(-.15);
                    robot.leftRearDrive.setPower(-.15);
                    robot.rightRearDrive.setPower(.15);
                    sleep (100);

                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                    sleep(100);
                }
            }

        }

        if(State==11) {
            telemetry.addData("State",State);
            telemetry.update();
            x=1;
            robot.stageTwo.setPower(.6);
            sleep(1000);
            robot.stageTwo.setPower(0);

            robot.leftFrontDrive.setPower(-.2);
            robot.rightFrontDrive.setPower(-.2);
            robot.leftRearDrive.setPower(-.2);
            robot.rightRearDrive.setPower(-.2);
            sleep(450);
            robot.stageTwo.setPower(-.6);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            sleep(100);
            robot.leftFrontDrive.setPower(.45);
            robot.rightFrontDrive.setPower(.45);
            robot.leftRearDrive.setPower(.45);
            robot.rightRearDrive.setPower(.45);
            sleep(300);
            robot.stageTwo.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            sleep(100);
            robot.leftFrontDrive.setPower(-.25);
            robot.rightFrontDrive.setPower(-.25);
            robot.leftRearDrive.setPower(-.25);
            robot.rightRearDrive.setPower(-.25);
            sleep(200);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            sleep(100);

        }
        //forward, back up
    }

    public void extraGlyph()
    {
        robot.leftStageOne.setPower(1);
        robot.rightStageOne.setPower(1);

        robot.leftFrontDrive.setPower(-.45);
        robot.rightFrontDrive.setPower(-.45);
        robot.leftRearDrive.setPower(-.45);
        robot.rightRearDrive.setPower(-.45);
        sleep(2000);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        sleep (100);
        do {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            z = AngleUnit.DEGREES.normalize(angles.firstAngle);
            if (z < 0)
                z += 360;

            robot.leftFrontDrive.setPower(-.25);
            robot.rightFrontDrive.setPower(.25);
            robot.leftRearDrive.setPower(-.25);
            robot.rightRearDrive.setPower(.25);

        } while (z - startZ > 75 || z - startZ < 71);
        {
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            sleep(100);
        }

        try {
            do {
                robot.leftFrontDrive.setPower(.25);
                robot.rightFrontDrive.setPower(.25);
                robot.leftRearDrive.setPower(.25);
                robot.rightRearDrive.setPower(.25);


                telemetry.addData("cm1", "%.2f cm", robot.rangeSensor1.getDistance(DistanceUnit.CM));//add try catch

                telemetry.addData("cm2", "%.2f cm", robot.rangeSensor2.getDistance(DistanceUnit.CM));//add try catch

                telemetry.update();

            } while (robot.rangeSensor2.getDistance(DistanceUnit.CM) > 30);
        } catch (Exception e) {
            telemetry.addData("err", "distance2");//add try catch

            telemetry.update();

        }
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        sleep(100);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(-.2);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(-.2);
        sleep(50);
        //strafe until lined up
        do {
            robot.leftFrontDrive.setPower(.2);
            robot.rightFrontDrive.setPower(-.2);
            robot.leftRearDrive.setPower(-.25);
            robot.rightRearDrive.setPower(.25);
            telemetry.addData("cm1", "%.2f cm", robot.rangeSensor1.getDistance(DistanceUnit.CM));//add try catch

            telemetry.addData("cm2", "%.2f cm", robot.rangeSensor2.getDistance(DistanceUnit.CM));//add try catch

            telemetry.update();
        } while (robot.rangeSensor2.getDistance(DistanceUnit.CM) > 12);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        sleep(100);

        robot.leftFrontDrive.setPower(.2);
        robot.rightFrontDrive.setPower(-.2);
        robot.leftRearDrive.setPower(-.2);
        robot.rightRearDrive.setPower(.2);
        sleep(150);

        robot.stageTwo.setPower(.6);
        sleep(500);
        robot.stageTwo.setPower(0);

        robot.leftFrontDrive.setPower(-.2);
        robot.rightFrontDrive.setPower(-.2);
        robot.leftRearDrive.setPower(-.2);
        robot.rightRearDrive.setPower(-.2);
        sleep(200);
        robot.stageTwo.setPower(-.6);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        sleep(200);
        robot.leftStageOne.setPower(0);
        robot.rightStageOne.setPower(0);
        robot.leftFrontDrive.setPower(.45);
        robot.rightFrontDrive.setPower(.45);
        robot.leftRearDrive.setPower(.45);
        robot.rightRearDrive.setPower(.45);
        sleep(300);
        robot.stageTwo.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        sleep(200);
        robot.leftFrontDrive.setPower(-.25);
        robot.rightFrontDrive.setPower(-.25);
        robot.leftRearDrive.setPower(-.25);
        robot.rightRearDrive.setPower(-.25);
        sleep(200);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        sleep(200);
        robot.leftFrontDrive.setPower(.25);
        robot.rightFrontDrive.setPower(.25);
        robot.leftRearDrive.setPower(.25);
        robot.rightRearDrive.setPower(.25);
        sleep(200);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        sleep (200);

    }
}