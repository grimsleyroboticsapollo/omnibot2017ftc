/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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
@TeleOp(name="Omnibot TestOp", group="Omnibot")
//@Disabled
public class teleOp_demo2 extends OpMode{

    /* Declare OpMode members. */
    HardwareOmnibot robot       = new HardwareOmnibot();
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;// sets rate to move servo
    boolean changeMoveMemo;
    boolean moveScheme = false;
    boolean rampState = false;
    boolean rampBack = false;
    int rampUp = 0;
    double rampDefault = .5;
    double bumperDefault = .7;
    double ShootMotor = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    ElapsedTime timer = new ElapsedTime();
    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");
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
        double leftY = -gamepad1.left_stick_y;
        double rightY = -gamepad1.right_stick_y;
        double leftX = -gamepad1.left_stick_x;
        double rightX = -gamepad1.right_stick_x;
        boolean changeMove = gamepad1.a;
        boolean turnLeft = gamepad1.left_bumper;
        boolean turnRight = gamepad1.right_bumper;
        boolean bumper = gamepad1.b;
        boolean feed = gamepad2.a;
        boolean shoot = gamepad2.b;
        boolean ramp = gamepad2.left_bumper;
        boolean back = gamepad2.right_bumper;
        boolean Color1 = gamepad2.x;
        if(changeMove && !changeMoveMemo){
            moveScheme = !moveScheme;
        }
        if(moveScheme){
            if(turnLeft){
                robot.leftFrontMotor.setPower(1);
                robot.leftBackMotor.setPower(1);
                robot.rightFrontMotor.setPower(-1);
                robot.rightBackMotor.setPower(-1);

            }else if(turnRight) {
                robot.leftFrontMotor.setPower(-1);
                robot.leftBackMotor.setPower(-1);
                robot.rightFrontMotor.setPower(1);
                robot.rightBackMotor.setPower(1);


            }else{
                robot.leftFrontMotor.setPower(leftY);
                robot.rightFrontMotor.setPower(leftX);
                robot.leftBackMotor.setPower(leftX);
                robot.rightBackMotor.setPower(leftY);
            }

        }else{
            robot.leftFrontMotor.setPower(-leftY);
            robot.rightFrontMotor.setPower(-rightY);
            robot.leftBackMotor.setPower(-leftY);
            robot.rightBackMotor.setPower(-rightY);
        }
        if (feed){
            robot.FeedMotor.setPower(.9);
        } else {
            robot.FeedMotor.setPower(0);
        }

        if (shoot) {
            ShootMotor = 1;
        } else {
            if(ShootMotor > 0.01){
                ShootMotor -= .25 * ShootMotor;
            } else {
                ShootMotor = 0;
            }
        }
        robot.rightShootMotor.setPower(ShootMotor);
        robot.leftShootMotor.setPower(-ShootMotor);

        if(ramp && !rampState){
            rampUp++;
        } else if(back && !rampBack){
            rampUp--;
        }
        if(rampUp > 2){
            rampUp = 0;
        } else if(rampUp < 0){
            rampUp = 2;
        }
        switch (rampUp){
            default:
                robot.Ramp.setPosition(rampDefault);
                break;
            case 0:
                robot.Ramp.setPosition(rampDefault-.25);
                break;
            case 1:
                robot.Ramp.setPosition(rampDefault+.25);
                break;
            case 2:
                robot.Ramp.setPosition(rampDefault+.1);
                break;
        }
        changeMoveMemo = changeMove;
        rampState = ramp;
        rampBack = back;
    }
    /*
     * Code to run ONCE after the driver hits STOP
    */
    @Override
    public void stop() {
    }

}
