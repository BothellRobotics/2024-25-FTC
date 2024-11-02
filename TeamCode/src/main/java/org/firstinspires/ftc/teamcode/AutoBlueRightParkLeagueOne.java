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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Blue Right Parking", group="Robot")
public class AutoBlueRightParkLeagueOne extends LinearOpMode {

    static final long DELAYED_START = 3000;
    static final boolean IS_PARK_CLOSE_TO_WALL = false;

    static final double PARK_CLOSE_TO_WALL = 50.0;
    static final double PARK_AWAY_FROM_WALL = 35.0;

    /* Declare OpMode members. */
    //private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    //private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor slideRight = null;
    public DcMotor slideLeft = null;
    public Servo bottomServo;
    public Servo topServo;
    public Servo leftServo;
    public Servo rightServo;
    double clawOffset = 0;
    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 5.512 ;     // For figuring circumference
    static final double     PULLEY_WHEEL_DIAMETER_INCHES = 1.5;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     SLIDE_COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final int    UPPER_LIMIT_ENCODER = 4200 ;
    static final double INCREMENT   = 0.003;
    static final double     TOP_SERVO_INIT_POS      = 0.31;
    static final double BOTTOM_SERVO_INIT_POS = 0.30;

    static final double TOP_SERVO_PICKUP_POS = 0.41;
    static final double TOP_SERVO_HOVER_POS = 0.38;
    static final double TOP_SERVO_OUT_OF_CAGE_POS = 0.365;
    static final double TOP_SERVO_SAMPLE_PICKUP_POS = 0.405;

    static final double LEFT_SERVO_INIT_POS = 0.26;
    static final double RIGHT_SERVO_INIT_POS = 0.0;

    static final double LEFT_SERVO_HOVER_POS = 0.22;
    static final double RIGHT_SERVO_HOVER_POS = 0.04;

    static final double LEFT_SERVO_CLOSE_POS = 0.176;
    static final double RIGHT_SERVO_CLOSE_POS = 0.089;

    static final double LEFT_SERVO_SLIGHT_OPEN_POS = 0.185;
    static final double RIGHT_SERVO_SLIGHT_OPEN_POS = 0.08;

    static final double ARM_UP_BAR = 16.0;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.

        //leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        //rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        //leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        //leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");

        bottomServo = hardwareMap.get(Servo.class, "bottomServo");
        topServo = hardwareMap.get(Servo.class, "topServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");


        bottomServo.setPosition(BOTTOM_SERVO_INIT_POS);
        topServo.setPosition(TOP_SERVO_INIT_POS);
        leftServo.setPosition(LEFT_SERVO_CLOSE_POS);
        rightServo.setPosition(RIGHT_SERVO_CLOSE_POS);

        slideRight.setDirection(DcMotor.Direction.REVERSE);
        slideLeft.setDirection(DcMotor.Direction.FORWARD);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean isSlideRaised = false;
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double bottomServoPos = 0.0;
        double topServoPos = 0.0;
        double leftServoPos = 0.0;
        double rightServoPos = 0.0;
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d ",
                            rightBackDrive.getCurrentPosition(),
                            leftBackDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        slideLeft.setPower(0);
        slideRight.setPower(0);

        sleep(DELAYED_START );
        encoderDrive(0.2, 17, 17, 10.0);
        encoderDrive(0.2, 28, -28, 10.0);

        if(IS_PARK_CLOSE_TO_WALL)
            encoderDrive(0.2, PARK_CLOSE_TO_WALL, PARK_CLOSE_TO_WALL, 10.0);
        else
            encoderDrive(0.2, PARK_AWAY_FROM_WALL, PARK_AWAY_FROM_WALL, 10.0);

        encoderDrive(0.2, 28, -28, 10.0);
        encoderDrive(0.2, 21, 21, 10.0);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100);  // pause to display final telemetry message.
    }

    public void linearSlideDrive(DcMotor slideLeft, DcMotor slideRight,
                                 double leftSpeed, double rightSpeed,
                                 double heightInches, double timeoutS) {
        int newSlideTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newSlideTarget = slideLeft.getCurrentPosition() - (int) (heightInches * SLIDE_COUNTS_PER_INCH);

            if(newSlideTarget > UPPER_LIMIT_ENCODER){
                newSlideTarget = UPPER_LIMIT_ENCODER;
            }

            if(newSlideTarget < 0){
                newSlideTarget = 0;
            }
            slideLeft.setTargetPosition(newSlideTarget);
            slideRight.setTargetPosition(newSlideTarget);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            slideLeft.setPower(Math.abs(leftSpeed));
            slideRight.setPower(Math.abs(rightSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((slideRight.isBusy()) || (slideLeft.isBusy()))) {
                telemetry.addData("Currently left slide is",  " at %b",
                        slideLeft.isBusy());
                telemetry.addData("Currently right slide is",  " at %b",
                        slideRight.isBusy());
                // Display it for the driver.

                telemetry.addData("Currently left slide at",  " at %7d",
                        slideLeft.getCurrentPosition());
                telemetry.addData("Currently right slide at",  " at %7d",
                        slideRight.getCurrentPosition());
                telemetry.update();
            }

            telemetry.addData("Exited for loop, left side at",  " at %7d",
                    slideLeft.getCurrentPosition());
            telemetry.addData("Exited for loop, right side at",  " at %7d",
                    slideRight.getCurrentPosition());
            telemetry.update();
            // Stop all motion;
            slideLeft.setPower(0);
            slideRight.setPower(0);

            // Turn off RUN_TO_POSITION
            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move.
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            //newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);
            //leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            //rightFrontDrive.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            //leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            //leftFrontDrive.setPower(Math.abs(speed));
            //rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newRightBackTarget, newLeftBackTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.addData("Current Power ",  " at LB : %4.2f RB : %4.2f",
                        rightBackDrive.getPower(), leftBackDrive.getPower());
                telemetry.update();
            }

            // Stop all motion;
            //leftFrontDrive.setPower(0);
            //rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            //leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move.
        }
    }

    public void doubleEncoderDrive(DcMotor slideLeft,DcMotor slideRight,
                                   double leftSpeed, double rightSpeed,
                                   double heightInches, double timeoutS) {
        int newSlideTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if(leftSpeed >= 0) {
                newSlideTarget = slideLeft.getCurrentPosition() + (int) (heightInches * COUNTS_PER_INCH);
            } else {
                newSlideTarget = 0;
            }
            if(newSlideTarget > UPPER_LIMIT_ENCODER){
                newSlideTarget = UPPER_LIMIT_ENCODER;
            }
            slideLeft.setTargetPosition(newSlideTarget);
            slideRight.setTargetPosition(newSlideTarget);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            slideLeft.setPower(Math.abs(leftSpeed));
            slideRight.setPower(Math.abs(rightSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((slideRight.isBusy()) || (slideLeft.isBusy()))) {
                telemetry.addData("Currently left slide is",  " at %b",
                        slideLeft.isBusy());
                telemetry.addData("Currently right slide is",  " at %b",
                        slideRight.isBusy());
                // Display it for the driver.

                telemetry.addData("Currently left slide at",  " at %7d",
                        slideLeft.getCurrentPosition());
                telemetry.addData("Currently right slide at",  " at %7d",
                        slideRight.getCurrentPosition());
                telemetry.update();
            }

            telemetry.addData("Exited for loop, left side at",  " at %7d",
                    slideLeft.getCurrentPosition());
            telemetry.addData("Exited for loop, right side at",  " at %7d",
                    slideRight.getCurrentPosition());
            telemetry.update();
            // Stop all motion;
            slideLeft.setPower(0);
            slideRight.setPower(0);

            // Turn off RUN_TO_POSITION
            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move.
        }
    }
}