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

@TeleOp(name="TeleOpsBeginner", group="TeleOps")
//@Disabled
public class TeleOps extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
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
    static final double INCREMENT   = 0.001;

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double bottomServoPos = 0.0;
        double topServoPos = 0.0;
        double leftServoPos = 0.0;
        double rightServoPos = 0.0;

        bottomServo = hardwareMap.get(Servo.class, "bottomServo");
        topServo = hardwareMap.get(Servo.class, "topServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        bottomServo.setPosition(0.31);
        topServo.setPosition(0.32);
        leftServo.setPosition(0.26);
        rightServo.setPosition(0.0);

        // Define and Initialize Motors
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            slideLeft.setPower(0);
            slideRight.setPower(0);

            double forwardSpeed = 0.5;
            // Use gamepad left & right Bumpers to open and close the claw


            if (gamepad1.dpad_left) {
                rightFront.setPower(0.8);
                leftFront.setPower(-forwardSpeed);
                rightBack.setPower(forwardSpeed);
                leftBack.setPower(-forwardSpeed);
            }
            if (gamepad1.dpad_right) {
                rightFront.setPower(-0.8);
                leftFront.setPower(forwardSpeed);
                rightBack.setPower(-forwardSpeed);
                leftBack.setPower(forwardSpeed);
            }

            if (gamepad1.right_stick_y != 0) {
                leftFront.setPower(gamepad1.right_stick_y * 0.5);
                leftBack.setPower(gamepad1.right_stick_y * 0.5);
                rightFront.setPower(-gamepad1.right_stick_y * 0.5);
                rightBack.setPower(-gamepad1.right_stick_y * 0.5);
            }

            if (gamepad1.dpad_down) {
                rightFront.setPower(-0.8);
                leftFront.setPower(-forwardSpeed);
                rightBack.setPower(forwardSpeed);
                leftBack.setPower(forwardSpeed);
            }

            if (gamepad1.dpad_up) {
                rightFront.setPower(0.8);
                leftFront.setPower(forwardSpeed);
                rightBack.setPower(-forwardSpeed);
                leftBack.setPower(-forwardSpeed);
            }

            if(gamepad1.right_bumper) {
                encoderDrive(slideLeft, -0.4,  10,  10, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            }

            //Left Slide goes down
            if(gamepad1.left_bumper) {
                encoderDrive(slideLeft, -0.4,  10,  10, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            }

            //Right Slide goes up for Right Trigger
            if(gamepad1.right_trigger > 0.1) {
                encoderDrive(slideRight, 0.7,  10,  10, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            }

            //Left Slide goes up for Left Trigger
            if(gamepad1.left_trigger > 0.1) {
                encoderDrive(slideLeft, 0.625,  10,  10, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            }
/*
            if(gamepad1.x){

                doubleEncoderDrive(slideLeft, slideRight, 0.625, 0.7, 43, 10.0);
            }
            if(gamepad1.b){

                doubleEncoderDrive(slideLeft, slideRight, -0.4, -0.4, 43, 10.0);
            }
*/


            if (gamepad2.b) {
                bottomServoPos = bottomServo.getPosition();
                bottomServoPos += INCREMENT;
                if(bottomServoPos >= 0.5)
                    bottomServoPos = 0.5;
                if(topServo.getPosition() > 0.33)
                    bottomServo.setPosition(bottomServoPos);
                else
                    bottomServo.setPosition(0.31);
            }
            if (gamepad2.x){
                bottomServoPos = bottomServo.getPosition();
                bottomServoPos -= INCREMENT;
                if(bottomServoPos <= 0.0)
                    bottomServoPos = 0.0;

                bottomServo.setPosition(bottomServoPos);
            }

            if (gamepad2.a) {   //Top Servo going down
                topServoPos = topServo.getPosition();
                topServoPos += INCREMENT;
                if(topServoPos >= 0.375)
                    topServoPos = 0.375;
                topServo.setPosition(topServoPos);
            }
            if (gamepad2.y){    //Top Servo going UP
                topServoPos = topServo.getPosition();
                topServoPos -= INCREMENT;
                if(topServoPos <= 0.32)
                    topServoPos = 0.32;

                topServo.setPosition(topServoPos);
            }

            if(gamepad2.right_trigger > 0) {  //Right Servo Opening
                rightServoPos = rightServo.getPosition();
                rightServoPos -= INCREMENT;
                if(rightServoPos <= 0.0)
                    rightServoPos = 0.0;

                rightServo.setPosition(rightServoPos);
            }

            if(gamepad2.left_trigger > 0) { //Left Servo Opening
                leftServoPos = leftServo.getPosition();
                leftServoPos += INCREMENT;
                if(leftServoPos >= 0.26)
                    leftServoPos = 0.26;
                leftServo.setPosition(leftServoPos);
            }

            if(gamepad2.right_bumper) { //Right Servo Closing
                rightServoPos = rightServo.getPosition();
                rightServoPos += INCREMENT;
                if(rightServoPos >= 0.06)
                    rightServoPos = 0.06;

                rightServo.setPosition(rightServoPos);
            }

            if(gamepad2.left_bumper) {  //Left Servo Closing
                leftServoPos = leftServo.getPosition();
                leftServoPos -= INCREMENT;
                if(leftServoPos <= 0.19)
                    leftServoPos = 0.19;

                leftServo.setPosition(leftServoPos);
            }

            if (gamepad1.left_stick_button) {
                // Keep stepping up until we hit the max value.
                leftServoPos = leftServo.getPosition();
                leftServoPos += INCREMENT;
                if(leftServoPos >= 0.07)
                    leftServoPos = 0.07;
                leftServo.setPosition(leftServoPos);
            }
            if (gamepad1.right_stick_button){
                rightServoPos = rightServo.getPosition();
                rightServoPos -= INCREMENT;
                if(rightServoPos <= 0.0)
                    rightServoPos = 0.0;

                rightServo.setPosition(rightServoPos);
            }

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);

            // Send telemetry message to signify robot running;
            telemetry.addData("Left Slide Currently at",  " at %7d", slideLeft.getCurrentPosition());
            telemetry.addData("Right Slide Currently at",  " at %7d ", slideRight.getCurrentPosition());

            telemetry.addData("claw",  "  bottomServoPos  = %.2f", bottomServo.getPosition());
            telemetry.addData("topServoPos = ",  "%.2f", topServo.getPosition());
            telemetry.addData("right", "%.2f", right);

            telemetry.addData("Left Servo Position",  " at %7f", leftServo.getPosition());
            telemetry.addData("Right Servo Position",  " at %7f", rightServo.getPosition());

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

                // Display it for the driver.

                telemetry.addData("Currently left slide at",  " at %7d",
                        slideLeft.getCurrentPosition());
                telemetry.addData("Currently right slide at",  " at %7d",
                        slideRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            slideLeft.setPower(0);
            slideRight.setPower(0);

            // Turn off RUN_TO_POSITION
            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
