/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
/**
 * Instead of each Op Mode class redefining the robot's hardware resources within its implementation,
 * This RobotHardware class has a given robot's component resources defined and set up all in one place.
 * It also has convenience methods like getSensorDistance(), setArmPower(), setHandPosition(), etc. that work for that robot.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */
public class RobotHardware {

    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define all the HardwareDevices (Motors, Servos, etc.). Make them private so they can't be accessed externally.
    private DcMotor sliderMotor;
    private Servo grabber;
    private Servo boxServo;
    private DcMotor intake;
    private DcMotor intakearm;

    // Hardware device constants.  Make them public so they can be used by the calling OpMode, if needed.
    static final double COUNTS_PER_MOTOR_REV = 560;     // Assumes 20:1 gear reduction
    // See https://docs.revrobotics.com/duo-control/sensors/encoders/motor-based-encoders
    static final double ENCODER_COUNT_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;

    static final double DEFAULT_MOTOR_SPEED = .4;
    static final double  HEADING_THRESHOLD = 1.0 ; // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    static final double MID_SERVO       =  0.5 ;
    static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    static final double ARM_UP_POWER    =  0.45 ;
    static final double ARM_DOWN_POWER  = -0.45 ;
    static final double MAX_POTENTIOMETER_ANGLE = 270;

    static final double SENSOR_DISTANCE_OUT_OF_RANGE = 20;

    //Update these IMU parameters for your robot.
    static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    /**
     * You can set the arm positions using angles and/or potentiometer voltage.
     * Tune these values for your robot's actual values.
     */
    static final double ARM_PARKED_ANGLE = 0;
    static final double ARM_PIXEL_PICKUP_ANGLE = 200;
    static final double ARM_BACKDROP_ANGLE = 100;
    static final double ARM_PARKED_VOLTAGE = 0;
    static final double ARM_PIXEL_PICKUP_VOLTAGE = 3;
    static final double ARM_BACKDROP_VOLTAGE = 1.4;

    /**
     * The one and only constructor requires a reference to a LinearOpMode.
     * @param opmode
     */
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initSliderMotor();
        initGrabberServo();
        initIntakeMotor();
        initIntakeArmMotor();
        initBoxServo();

//        myOpMode.telemetry.addData(">", "Hardware Initialized");
//        myOpMode.telemetry.update();
    }

    private void initSliderMotor() {
        sliderMotor = myOpMode.hardwareMap.get(DcMotor.class, "SliderMotor");
        sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initGrabberServo() {
        grabber = myOpMode.hardwareMap.get(Servo.class, "grabberservo");
    }

    private void initIntakeMotor() {
        intake = myOpMode.hardwareMap.get(DcMotor.class, "IntakeMotor");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initIntakeArmMotor() {
        intakearm = myOpMode.hardwareMap.get(DcMotor.class, "IntakeArm");
    }

    private void initBoxServo() {
        boxServo = myOpMode.hardwareMap.get(Servo.class, "boxservo");
        myOpMode.telemetry.addData("boxServo", boxServo.getPosition());
        myOpMode.telemetry.update();
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     */
    public void moveSliderUp() {
        int currentPosition = sliderMotor.getCurrentPosition();
        sliderMotor.setTargetPosition(currentPosition + 120);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(1);
    }

    public void moveSliderDown() {
        int currentPosition = sliderMotor.getCurrentPosition();
        sliderMotor.setTargetPosition(currentPosition - 120);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(-1);
    }

    public void openGrabber() {
        grabber.setPosition(0.8);
    }

    public void closeGrabber() {
        grabber.setPosition(0.2);
    }

    public void intake() {
        intake.setPower(1);
    }

    public void outtake() {
        intake.setPower(-1);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void intakeArmUp() {
        intakearm.setPower(0.25);
    }

    public void intakeArmDown() {
        intakearm.setPower(-0.25);
    }

    public void boxServoUp() {
        boxServo.setPosition(1);
    }

    public void boxServoDown() {
        boxServo.setPosition(.5);
    }

    public void setRunModeForAllArms(DcMotor.RunMode runMode) {
        //leftArm.setMode(runMode);
        //rightArm.setMode(runMode);
    }

    /**
     * Set up telemetry to output this:
     *    <targetCaption> : <targetValue>
     *    <currentCaption> : <currentValue>
     * The Telemetry.Item for the currentValue is returned so the caller can keep updating it
     * using setValue().
     *
     * @param targetCaption
     * @param targetValue
     * @param currentCaption
     * @param currentValue
     * @return currentItem
     */
    private Telemetry.Item setupArmPositionTelemetry(String targetCaption, double targetValue, String currentCaption, double currentValue) {
        Telemetry telemetry = myOpMode.telemetry;
        telemetry.addData(targetCaption, targetValue);
        Telemetry.Item currentItem = telemetry.addData(currentCaption, currentValue);
        telemetry.update(); //Allow driver station to be cleared before display.
        telemetry.setAutoClear(false); //Henceforth updates should not clear display.

        return currentItem;
    }
}