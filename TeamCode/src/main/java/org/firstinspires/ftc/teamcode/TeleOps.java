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

    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

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

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        /*leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        rightClaw = hardwareMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);*/

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

           /* if (gamepad1.right_stick_y > 0) {
                rightFront.setPower(-0.8);
                leftFront.setPower(forwardSpeed);
                rightBack.setPower(forwardSpeed);
                leftBack.setPower(forwardSpeed);
            }
            if (gamepad1.right_stick_y < 0) {
                rightFront.setPower(0.8);
                leftFront.setPower(-forwardSpeed);
                rightBack.setPower(-forwardSpeed);
                leftBack.setPower(-forwardSpeed);
            }*/
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
/*
            //goes down
            if (gamepad1.right_bumper) {
                slideRight.setPower(-0.4);
                telemetry.addData("Currently at",  " at %7d ", slideRight.getCurrentPosition());
                telemetry.update();
            }
            //goes down
            if (gamepad1.left_bumper) {
                slideLeft.setPower(-0.4);
                telemetry.addData("Currently at",  " at %7d", slideLeft.getCurrentPosition());
                telemetry.update();
            }

            //goes up
            if (gamepad1.right_trigger > 0.5) {
                slideRight.setPower(0.7);
                telemetry.addData("Currently at",  " at %7d ", slideRight.getCurrentPosition());
                telemetry.update();
            }

            //goes up
            if (gamepad1.left_trigger > 0.5) {
                slideLeft.setPower(0.625);
                telemetry.addData("Currently at",  " at %7d", slideLeft.getCurrentPosition());
                telemetry.update();
            }*/

            if (gamepad1.y) {
                slideLeft.setPower(0.625);
                slideRight.setPower(0.7);
            }

            if (gamepad1.a) {
                slideLeft.setPower(-0.4);
                slideRight.setPower(-0.4);
            }

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            /*leftClaw.setPosition(MID_SERVO + clawOffset);
            rightClaw.setPosition(MID_SERVO - clawOffset);*/

            // Use gamepad buttons to move arm up (Y) and down (A)
           /* if (gamepad1.y)
                rightFront.setPower(ARM_UP_POWER);
            else if (gamepad1.a)
                rightFront.setPower(ARM_DOWN_POWER);
            else
                rightFront.setPower(0.0);*/

            // Send telemetry message to signify robot running;
            telemetry.addData("Left Slide Currently at",  " at %7d", slideLeft.getCurrentPosition());
            telemetry.addData("Right Slide Currently at",  " at %7d ", slideRight.getCurrentPosition());
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
