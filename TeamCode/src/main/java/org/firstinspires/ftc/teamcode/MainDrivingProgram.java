/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.DirectionalPower;
import org.firstinspires.ftc.teamcode.util.Vector2;
import org.threeten.bp.Instant;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "MainDrivingProgram", group = "Linear OpMode")
public class MainDrivingProgram extends LinearOpMode {

    // Variable Declaration
    ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    private final double DEADZONE = 0.1;
    private final boolean DEBUG = true;
    //Naming
    private final String frontLeftMotorName = "frontLeft";
    private final String backLeftMotorName = "backLeft";
    private final String frontRightMotorName = "frontRight";
    private final String backRightMotorName = "backRight";

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, frontLeftMotorName);
        backLeftDrive = hardwareMap.get(DcMotor.class, backLeftMotorName);
        frontRightDrive = hardwareMap.get(DcMotor.class, frontRightMotorName);
        backRightDrive = hardwareMap.get(DcMotor.class, backRightMotorName);


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //init variables
        Vector2 dir = new Vector2(0,0);
        int loopCount = 0;
        double rotation;
        boolean inverted = false;
        double power = 1;
        DirectionalPower dirPower = new DirectionalPower(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        //Timer vars
        long currentTime = System.currentTimeMillis();
        long flywheelSleepTimer = 0L;
        long invertSleepTimer = 0L;
        long driveSpeedSleepTimer = 0L;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        waitForStart();
        runtime.reset();

       //Main runtime loop
        while (opModeIsActive()) {
            loopCount++;
            //Delta Time System
            long deltaTime = System.currentTimeMillis() - currentTime;
            currentTime = System.currentTimeMillis();

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //Get direction
            dir.x = gamepad1.left_stick_x;
            dir.y = -gamepad1.left_stick_y;
            //Check deadzone
            if(dir.magnitude() < DEADZONE) dir.zero();
            //Get the rotation
            rotation = gamepad1.right_stick_x;
            if(Math.abs(rotation) < DEADZONE) rotation = 0;
            //See if inverted
            if(inverted){
                dir.invert();
            }
            //Apply
            dirPower.drive(dir, power, rotation, telemetry, false);

            //Handle drive speed
            if (gamepad1.right_trigger > DEADZONE && gamepad1.left_trigger > DEADZONE){
                //Pressing both triggers returns speed to max
                power = 1;
            } else if (gamepad1.left_trigger > DEADZONE && driveSpeedSleepTimer <= 0){
                power -= 0.02;
                driveSpeedSleepTimer = 50;
            } else if (gamepad1.right_trigger > DEADZONE && driveSpeedSleepTimer <= 0){
                power += 0.02;
                driveSpeedSleepTimer = 50;
            }

            //Inversion
            if (gamepad1.right_bumper && gamepad1.left_bumper && invertSleepTimer <= 0){
                inverted = !inverted;
                invertSleepTimer = 500;
            }

            //Update Sleep Timer
            flywheelSleepTimer -= deltaTime;
            invertSleepTimer -= deltaTime;
            driveSpeedSleepTimer -= deltaTime;

            if(DEBUG) {
                //Debug code goes here
                telemetry.addData("Current Time", Instant.now());
                telemetry.addData("Loop Count", loopCount);
                telemetry.addData("Sleep Timer", flywheelSleepTimer);
                telemetry.addData("DeltaTime", deltaTime);
                telemetry.addData("Rotate Speed", rotation);
                telemetry.addData("Gamepad 1 Left Stick Y", gamepad1.left_stick_y);
                telemetry.addData("Gamepad 1 Left Stick X", gamepad1.left_stick_x);
                telemetry.addData("Gamepad 1 Right Stick Y", gamepad1.right_stick_y);
                telemetry.addData("Gamepad 1 Right Stick X", gamepad1.right_stick_x);
                telemetry.addData("Gamepad 2 Right Bumper", gamepad2.right_bumper);
                telemetry.addData("Gamepad 2 Right Trigger", gamepad2.right_trigger);
                telemetry.addData("Gamepad 2 Left Trigger", gamepad2.left_trigger);
                telemetry.addData("Gamepad 2 DPad Up", gamepad2.dpad_up);
                telemetry.addData("Gamepad 2 DPad Down", gamepad2.dpad_down);
                dirPower.debugTelemetry(telemetry);
                telemetry.update();
            }

        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
