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
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    private Servo boxServo;
    private DcMotor intake;
    private DcMotor intakearm;
    private ColorSensor boxcolor;
    private Servo leftBoxServo;
    private Servo rightBoxServo;
    private DcMotor rightintakemotor;
    private DcMotor leftintakemotor;

    private int initialIntakeMotorPosition = 0;
    private int initialSliderMotorPosition = 0;

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
        initIntakeMotor();
        initIntakeArmMotor();
        initBoxServo();
        //initSensors();
        initBoxServos();

//        myOpMode.telemetry.addData(">", "Hardware Initialized");
//        myOpMode.telemetry.update();
    }

    private void initSliderMotor() {
        sliderMotor = myOpMode.hardwareMap.get(DcMotor.class, "SliderMotor");
        sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initialSliderMotorPosition = sliderMotor.getCurrentPosition();
    }

    private void initIntakeMotor() {
        rightintakemotor = myOpMode.hardwareMap.get(DcMotor.class, "RightIntakeMotor");
        rightintakemotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftintakemotor = myOpMode.hardwareMap.get(DcMotor.class, "LeftIntakeMotor");
        leftintakemotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initIntakeArmMotor() {
        intakearm = myOpMode.hardwareMap.get(DcMotor.class, "IntakeArm");
        intakearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakearm.setDirection(DcMotorSimple.Direction.REVERSE);
        initialIntakeMotorPosition = intakearm.getCurrentPosition();
    }

    private void initBoxServos() {
        leftBoxServo = myOpMode.hardwareMap.get(Servo.class, "LeftBoxServo");
        rightBoxServo = myOpMode.hardwareMap.get(Servo.class, "RightBoxServo");
    }

    private void initBoxServo() {
        boxServo = myOpMode.hardwareMap.get(Servo.class, "boxservo");
        myOpMode.telemetry.addData("boxServo", boxServo.getPosition());
        myOpMode.telemetry.update();
    }

    private void initSensors() {
        boxcolor = myOpMode.hardwareMap.get(ColorSensor.class, "boxColorSensor");
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     */
    private int sliderPosition = 3500;

    public void moveSliderDownTeleop() {
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotor.setPower(1);
    }

    public void moveSliderDownAuto() {
        sliderMotor.setTargetPosition(initialSliderMotorPosition);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(1);
    }

    public void moveSliderUpTeleop() {
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotor.setPower(-1);
    }

    public void moveSliderUpAuto() {
        sliderMotor.setTargetPosition(initialSliderMotorPosition - 5800);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(1);
        while (sliderMotor.isBusy()) {

        }
    }


    public void stopSlider() {
        sliderMotor.setPower(0);
    }

    public void intake() {
        rightintakemotor.setPower(-0.6);
        leftintakemotor.setPower(-0.6);
    }

    public void outtake() {
        rightintakemotor.setPower(0.6);
        leftintakemotor.setPower(0.6);
    }

    public void stopIntake() {
        rightintakemotor.setPower(0);
        leftintakemotor.setPower(0);
    }

    public void intakeArmUp() {
        //int currentPosition = intakearm.getCurrentPosition();
        intakearm.setTargetPosition(initialIntakeMotorPosition);//currentPosition - 140);
        intakearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakearm.setPower(1);
        while (intakearm.isBusy()) {
            //do nothing
        }
        intakearm.setPower(0);
    }

    public void intakeArmDown() {
        intakearm.setTargetPosition(initialIntakeMotorPosition - 900);//currentPosition - 140);
        intakearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakearm.setPower(0.8);
        while (intakearm.isBusy()) {
            //do nothing
        }
        intakearm.setPower(0);
    }

    public void intakeArmMid() {
        intakearm.setTargetPosition(initialIntakeMotorPosition - 250);
        intakearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakearm.setPower(1);
        while (intakearm.isBusy()) {
            //do nothing
        }
        intakearm.setPower(0);
    }

    public void boxServoOpen() {
        leftBoxServo.setPosition(1);
        rightBoxServo.setPosition(0);
    }

    public void boxServoClose() {
        leftBoxServo.setPosition(0.3);
        rightBoxServo.setPosition(0.6);
    }

    public void boxServoUp() {
        boxServo.setPosition(0.95);
        myOpMode.sleep(2000);
    }

    public void boxServoDown() {
        boxServo.setPosition(.45);
    }

    public void setRunModeForAllArms(DcMotor.RunMode runMode) {
        //leftArm.setMode(runMode);
        //rightArm.setMode(runMode);
    }

    public boolean isRightColor(SpikeColor colorWeWant) {
        int blueThreshold = 4200;
        int redThreshold = 980;
        RGBAcolors colors = getSensorColors();

        if (colorWeWant == SpikeColor.BLUE) {
              return colors.getBlue() >= blueThreshold || colors.getRed() <= redThreshold;
        }

        if (colorWeWant == SpikeColor.RED) {
            return colors.getRed() >= redThreshold || colors.getBlue() <= blueThreshold;
        }

        return false;
    }

    public RGBAcolors getSensorColors() {
        int red = boxcolor.red();
        int green = boxcolor.green();
        int blue = boxcolor.blue();
        int alpha = boxcolor.alpha();
        myOpMode.telemetry.addData("Red",red);
        myOpMode.telemetry.addData("Green",green);
        myOpMode.telemetry.addData("Blue",blue);
        myOpMode.telemetry.update();


        return new RGBAcolors(red, green, blue, alpha);
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
        telemetry.update();
        telemetry.setAutoClear(false);

        return currentItem;
    }

}