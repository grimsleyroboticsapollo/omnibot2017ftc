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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


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
@TeleOp(name="Omnibot TeleOp", group="Omnibot")
//@Disabled
public class teleOp_demo extends OpMode{

    /* Declare OpMode members. */
    HardwareOmnibot robot       = new HardwareOmnibot();
    OpticalDistanceSensor odsSensor;  // Hardware Device Object
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;// sets rate to move servo
    boolean canResetTime = true;
    boolean changeMoveMemo;
    boolean lastShoot = false;
    boolean moveScheme = false;
    boolean rampState = false;
    boolean rampBack = false;
    int rampUp = 0;
    double rampDefault = .5;
    double bumperDefault = .7;
    double ShootMotor = 0;
    double startTime = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;
    ElapsedTime timer = new ElapsedTime();
    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 7;
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
        odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, false);
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
        timer.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        float hsvValues[] = {0F,0F,0F};
        Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
        double lightValue = 0.8028 * Math.pow(odsSensor.getLightDetected(), -0.999d); //in centimeters
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
        if(hsvValues[0] < 240 && hsvValues[0] > 180 && hsvValues[1] > .7 && lightValue < 4) {
            leftX = leftX * .25;
            leftY = leftY * .25;
            rightY = rightY * .25;
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
                robot.leftFrontMotor.setPower(leftX);
                robot.rightFrontMotor.setPower(leftY);
                robot.leftBackMotor.setPower(leftY);
                robot.rightBackMotor.setPower(leftX);
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
            if(canResetTime){
                startTime = timer.seconds();
                canResetTime = false;
            }
            if(timer.seconds() - startTime > .5) {
                ShootMotor = 1;
            }
            if(timer.seconds() - startTime > 2.5){
                robot.Ramp.setPosition(rampDefault-.25);
            } else {
                robot.Ramp.setPosition(rampDefault+.25);
            }
        } else {
            canResetTime = true;
            if(ShootMotor > 0.01){
                ShootMotor -= .25 * ShootMotor;
            } else {
                ShootMotor = 0;
            }
            if(ramp && !rampState){
                rampUp++;
            }
            if(back && !rampBack){
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
        }
        robot.rightShootMotor.setPower(ShootMotor);
        robot.leftShootMotor.setPower(-ShootMotor);


        lastShoot = shoot;
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
