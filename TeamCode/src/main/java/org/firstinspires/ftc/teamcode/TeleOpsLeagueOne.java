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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOps League 1", group="TeleOps")
//@Disabled
public class TeleOpsLeagueOne extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor slideRight = null;
    public DcMotor slideLeft = null;
    public Servo bottomServo;
    public Servo topServo;
    public Servo leftServo;
    public Servo rightServo;
    double clawOffset = 0;
    private ElapsedTime runtime = new ElapsedTime();


    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final int    UPPER_LIMIT_ENCODER = 4200 ;
    static final double INCREMENT   = 0.009;

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
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
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        boolean isSlideRaised = false;
        boolean isArmOpen = true;
        boolean isArmUp = true;
        double max;
        double bottomServoPos = 0.0;
        double topServoPos = 0.0;
        double leftServoPos = 0.0;
        double rightServoPos = 0.0;

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        bottomServo = hardwareMap.get(Servo.class, "bottomServo");
        topServo = hardwareMap.get(Servo.class, "topServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

       /* bottomServo.setPosition(BOTTOM_SERVO_INIT_POS);
        topServo.setPosition(TOP_SERVO_INIT_POS);
        leftServo.setPosition(LEFT_SERVO_INIT_POS);
        rightServo.setPosition(RIGHT_SERVO_INIT_POS);*/

        // Define and Initialize Motors

        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        slideRight.setDirection(DcMotor.Direction.REVERSE);
        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        bottomServo.setPosition(BOTTOM_SERVO_INIT_POS);
        topServo.setPosition(TOP_SERVO_INIT_POS);
        leftServo.setPosition(LEFT_SERVO_CLOSE_POS);
        rightServo.setPosition(RIGHT_SERVO_CLOSE_POS);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            // Output the safe vales to the motor drives.
            slideLeft.setPower(0);
            slideRight.setPower(0);

            double forwardSpeed = 0.5;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if (gamepad1.dpad_up) {
                leftFrontDrive.setPower(0.1);
                leftBackDrive.setPower(0.1);
                rightFrontDrive.setPower(0.1);
                rightBackDrive.setPower(0.1);
            }

            if (gamepad1.dpad_down) {
                leftFrontDrive.setPower(-0.1);
                leftBackDrive.setPower(-0.1);
                rightFrontDrive.setPower(-0.1);
                rightBackDrive.setPower(-0.1);
            }

            if (gamepad1.dpad_left) {
                leftFrontDrive.setPower(-0.1);
                leftBackDrive.setPower(0.1);
                rightFrontDrive.setPower(0.1);
                rightBackDrive.setPower(-0.1);
            }

            if (gamepad1.dpad_left) {
                leftFrontDrive.setPower(0.1);
                leftBackDrive.setPower(-0.1);
                rightFrontDrive.setPower(-0.1);
                rightBackDrive.setPower(0.1);
            }

            if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) {
                leftFrontDrive.setPower(0.0);
                leftBackDrive.setPower(0.0);
                rightFrontDrive.setPower(0.0);
                rightBackDrive.setPower(0.0);
            }
            //Reach the Top basket
            if (gamepad2.dpad_up && topServo.getPosition() < 0.33 && !isSlideRaised) {
                //Raise the slide
                doubleEncoderDrive(slideLeft, slideRight, 0.625, 0.7, 32, 10.0);

                isSlideRaised = true;
            }
            //Bring the slide all the way down
            if (gamepad2.dpad_up && isSlideRaised) {
                doubleEncoderDrive(slideLeft, slideRight, -0.6, -0.6, 32, 10.0);

                isSlideRaised = false;
                //Lower the slide
            }

            //Blue Bar to place the Specimen
            if (gamepad2.right_stick_button) {

                doubleDownDrive(slideLeft, slideRight, 0.2, 0.2, 3.6, 10.0);
                //Lower the slide slightly
                isSlideRaised = true;
            }
            if (isSlideRaised) {  //Hold the linear slide by powering the DC motors, when it went up. Otherwise slide will come down
                slideLeft.setPower(0.05);
                slideRight.setPower(0.05);
            }

            if (gamepad2.b) {//Moving bottom servo to the clockwise direction
                bottomServoPos = bottomServo.getPosition();
                bottomServoPos += 0.001;
                if (bottomServoPos >= 0.5)
                    bottomServoPos = 0.5;
                //When the pickup arm is horizontal then you are using the rotation of the arm
                if (topServo.getPosition() > TOP_SERVO_INIT_POS)
                    bottomServo.setPosition(bottomServoPos);
                    //If the arm is vertical the orientation/rotation of arm to the initial position
                else
                    bottomServo.setPosition(BOTTOM_SERVO_INIT_POS);
                //Move attachment to the right
            }
            if (gamepad2.x) { //Bottom servo moving counter clockwise direction
                bottomServoPos = bottomServo.getPosition();
                bottomServoPos -= 0.001;
                if (bottomServoPos <= 0.0)
                    bottomServoPos = 0.0;

                bottomServo.setPosition(bottomServoPos);
                //Move attachment to left
            }

            if (gamepad2.dpad_right && isArmUp) {   //Top Servo going down to pickup position
                topServo.setPosition(TOP_SERVO_PICKUP_POS);
                isArmUp = false;
                telemetry.addData("arm is DOWN is arm up = %b", isArmUp);
                telemetry.addData("topServoPos = ", "%.2f", topServo.getPosition());
                telemetry.update();
                sleep(1000);
            }
            if (gamepad2.dpad_right && !isArmUp) {
                topServo.setPosition(TOP_SERVO_INIT_POS);
                isArmUp = true;
                telemetry.addData("arm is UP is arm up = %b", isArmUp);
                telemetry.addData("topServoPos = ", "%.2f", topServo.getPosition());
                telemetry.update();
                sleep(1000);
            }


            //top servo/Sample Pickup lever hovers over the block & move the left and right
            //claws facing down.
            if (gamepad2.dpad_down) {
                topServo.setPosition(TOP_SERVO_HOVER_POS);

                leftServo.setPosition(LEFT_SERVO_HOVER_POS);
                rightServo.setPosition(RIGHT_SERVO_HOVER_POS);
            }

            /*if (gamepad2.dpad_up){    //top servo postition to leave cage
                topServoPos = topServo.getPosition();
                topServoPos -= INCREMENT;
                if(topServoPos <= TOP_SERVO_OUT_OF_CAGE_POS)
                    topServoPos = TOP_SERVO_OUT_OF_CAGE_POS;

                topServo.setPosition(topServoPos);
            }*/

            if (gamepad2.dpad_left && isArmOpen == true) { // opening and closing
                leftServo.setPosition(LEFT_SERVO_CLOSE_POS);
                rightServo.setPosition(RIGHT_SERVO_CLOSE_POS);
                isArmOpen = false;
                sleep(1000);
            }

            if (gamepad2.dpad_left && isArmOpen == false) {
                leftServo.setPosition(LEFT_SERVO_INIT_POS);
                rightServo.setPosition(RIGHT_SERVO_INIT_POS);
                isArmOpen = true;
                sleep(1000);
            }
           /*if(gamepad2.dpad_left) {  //Left Servo Closing Individual
                leftServoPos = leftServo.getPosition();
                leftServoPos -= INCREMENT;
                leftServo.setPosition(leftServoPos);
            }
            if(gamepad2.dpad_right) {  //Right Servo Closing Individual
                rightServoPos = rightServo.getPosition();
                rightServoPos += INCREMENT;
                rightServo.setPosition(rightServoPos);
            }*/
            if (gamepad2.left_stick_y > 0.1) {
                //left and right servo slight open
                rightServoPos = rightServo.getPosition();
                rightServoPos += INCREMENT;
                if (rightServoPos >= RIGHT_SERVO_SLIGHT_OPEN_POS)
                    rightServoPos = RIGHT_SERVO_SLIGHT_OPEN_POS;
                rightServo.setPosition(rightServoPos);

                leftServoPos = leftServo.getPosition();
                leftServoPos -= INCREMENT;
                if (leftServoPos <= LEFT_SERVO_SLIGHT_OPEN_POS)
                    leftServoPos = LEFT_SERVO_SLIGHT_OPEN_POS;

                leftServo.setPosition(leftServoPos);
            }

            if (gamepad2.right_stick_y > 0.1) {
                topServo.setPosition(TOP_SERVO_SAMPLE_PICKUP_POS);
            }

            if (gamepad2.left_stick_button) {
                doubleEncoderDrive(slideLeft, slideRight, 0.625, 0.7, ARM_UP_BAR, 10.0);
                isSlideRaised = true;
                //move arm to where blue bar is
            }

            if (gamepad2.back && isArmUp) {
                topServo.setPosition(0.33);
                telemetry.addData("topServoPos = ", "%.2f", topServo.getPosition());
                telemetry.update();
                isArmUp = false;
                sleep(2000);
            }

            if (gamepad2.back && !isArmUp) {
                topServo.setPosition(TOP_SERVO_INIT_POS);
                telemetry.addData("topServoPos = ", "%.2f", topServo.getPosition());
                telemetry.update();
                isArmUp = true;
                sleep(2000);
            }
            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            // telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            // Send telemetry message to signify robot running;
            telemetry.addData("Left Slide Currently at", " at %7d", slideLeft.getCurrentPosition());
            telemetry.addData("Right Slide Currently at", " at %7d ", slideRight.getCurrentPosition());

            telemetry.addData("claw", "  bottomServoPos  = %.2f", bottomServo.getPosition());
            telemetry.addData("topServoPos = ", "%.2f", topServo.getPosition());


            telemetry.addData("Left Servo Position", " at %7f", leftServo.getPosition());
            telemetry.addData("Right Servo Position", " at %7f", rightServo.getPosition());

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }


    public void encoderDrive(DcMotor slide, double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newSlideTarget;
        //int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if(speed >= 0)
                newSlideTarget = slide.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            else
                newSlideTarget = 0;
            slide.setTargetPosition(newSlideTarget);


            // Turn On RUN_TO_POSITION
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            slide.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (slide.isBusy())) {

                // Display it for the driver.

                telemetry.addData("Currently at",  " at %7d",
                        slide.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            slide.setPower(0);


            // Turn off RUN_TO_POSITION
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move.
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

    public void doubleDownDrive(DcMotor slideLeft,DcMotor slideRight,
                                   double leftSpeed, double rightSpeed,
                                   double heightInches, double timeoutS) {
        int newSlideTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newSlideTarget = slideLeft.getCurrentPosition() - (int) (heightInches * COUNTS_PER_INCH);

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

            //sleep(250);   // optional pause after each move.
        }
    }
}
