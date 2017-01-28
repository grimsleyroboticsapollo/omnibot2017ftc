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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Omnibot Autonomous Red", group="Omnibot")  // @Autonomous(...) is the other common choice
//@Disabled
public class AutoOmnibotRed extends OpMode
{
    /* Declare OpMode members. */
    // The IMU sensor object
    private double speed = .4;
    private double distanceThres = 1;
    BNO055IMU imu;
    OpticalDistanceSensor odsSensor;  // Hardware Device Object
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    HardwareOmnibot robot = new HardwareOmnibot();
    double bumperDefault = .5;
    boolean driveMotors = false;
    boolean colorMotors = false;
    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;
    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;
    ElapsedTime timer = new ElapsedTime();
    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, true);
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
        double lightValue = odsSensor.getLightDetected();
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double gyroDegrees = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        if((hsvValues[0] < 30 || hsvValues[0] > 330) && hsvValues[1] > .7 && driveMotors){
            colorMotors = true;
        }
        if (colorMotors){
            if(lightValue > (distanceThres + .007)){
                robot.rightFrontMotor.setPower(.25);
                robot.rightBackMotor.setPower(0);
                robot.leftBackMotor.setPower(.25);
                robot.leftFrontMotor.setPower(0);

                double startTime = timer.milliseconds();
                while((timer.milliseconds() - startTime) < 1250) {

                }
                robot.rightFrontMotor.setPower(-1);
                robot.rightBackMotor.setPower(0);
                robot.leftBackMotor.setPower(-1);
                robot.leftFrontMotor.setPower(0);

                startTime = timer.milliseconds();
                while((timer.milliseconds() - startTime) < 3000){

                }

                //make robot stop here
            } else {
                double degoff = 1 - ((double)((double) Math.abs(gyroDegrees - 45)) / 15);
                robot.rightBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);
                if(gyroDegrees < 45){
                    robot.leftBackMotor.setPower(speed);
                    robot.rightFrontMotor.setPower(speed*degoff);
                } else {
                    robot.leftBackMotor.setPower(speed*degoff);
                    robot.rightFrontMotor.setPower(speed);
                }
            }

        } else {
            if(driveMotors){
                robot.rightFrontMotor.setPower(0);
                robot.leftBackMotor.setPower(0);
                double degoff = 1 - ((double)((double) Math.abs(gyroDegrees - 45)) / 15);
                if(gyroDegrees < 45){
                    robot.leftFrontMotor.setPower(speed);
                    robot.rightBackMotor.setPower(speed*degoff);
                } else {
                    robot.leftFrontMotor.setPower(speed*degoff);
                    robot.rightBackMotor.setPower(speed);
                }
            } else if (lightValue > (distanceThres + .005)) {
                driveMotors = true;
                robot.leftFrontMotor.setPower(0);
                robot.rightBackMotor.setPower(0);
                robot.leftBackMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
            } else {
                if(gyroDegrees > 30 && gyroDegrees < 60) {//angle is within tolerance
                    if(distanceThres == 1){
                        distanceThres = lightValue;
                    }
                    telemetry.addData("Status", "within 45");
                    robot.rightBackMotor.setPower(0);
                    robot.leftFrontMotor.setPower(0);
                    double degoff = 1 - ((double)((double) Math.abs(gyroDegrees - 45)) / 15);
                    if(gyroDegrees < 45){
                        robot.leftBackMotor.setPower(speed);
                        robot.rightFrontMotor.setPower(speed*degoff);
                    } else {
                        robot.leftBackMotor.setPower(speed*degoff);
                        robot.rightFrontMotor.setPower(speed);
                    }
                } else {
                    if(gyroDegrees < 45){
                        telemetry.addData("Status", "below 45");
                        robot.leftFrontMotor.setPower(speed);
                        robot.leftBackMotor.setPower(speed);
                        robot.rightFrontMotor.setPower(-speed);
                        robot.rightBackMotor.setPower(-speed);
                    } else {
                        telemetry.addData("Status", "above 45");
                        robot.leftFrontMotor.setPower(-speed);
                        robot.leftBackMotor.setPower(-speed);
                        robot.rightFrontMotor.setPower(speed);
                        robot.rightBackMotor.setPower(speed);
                    }
                }

            }
        }
        telemetry.addData("Degrees: ",gyroDegrees);
        telemetry.addData("driveMotors: ",driveMotors);
        telemetry.addData("colorMotors: ",colorMotors);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
