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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    private DcMotor spinTakeMotor;
    private CRServo leftConveyServo;
    private CRServo rightConveyServo;
    private CRServo topConveyServo;
    private DcMotorEx rightShooterMotor;
    private DcMotorEx leftShooterMotor;

    // Hardware device constants.  Make them public so they can be used by the calling OpMode, if needed.
    static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    /**
     * The one and only constructor requires a reference to a LinearOpMode.
     * @param opmode
     */
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
        init();
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initSpinTakeMotor();
        initShooterMotors();
        initConveyorServos();
    }

    private void initSpinTakeMotor() {
        spinTakeMotor = myOpMode.
                hardwareMap.get(DcMotor.class, "SpinTake");
        spinTakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //spinTakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void initShooterMotors() {
        rightShooterMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "RightShoot");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "LeftShoot");
        leftShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initConveyorServos() {
        leftConveyServo = myOpMode.hardwareMap.get(CRServo.class, "LeftConvey");
        rightConveyServo = myOpMode.hardwareMap.get(CRServo.class, "RightConvey");
        rightConveyServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftConveyServo.setDirection(DcMotorSimple.Direction.FORWARD);

        topConveyServo = myOpMode.hardwareMap.get(CRServo.class, "TopConvey");
        topConveyServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }
/*    private void initSensors() {
        boxcolor = myOpMode.hardwareMap.get(ColorSensor.class, "boxColorSensor");
    }
*/

    public void startSpinTake() {
        spinTakeMotor.setPower(.85);
    }

    public void stopSpinTake() {
        spinTakeMotor.setPower(0);
    }

    public void startSideServos() {
        rightConveyServo.setPower(1);
        leftConveyServo.setPower(1);
    }

    public void stopSideServos() {
        rightConveyServo.setPower(0);
        leftConveyServo.setPower(0);
    }

    public void setTopConveyPower(double power) {
        topConveyServo.setPower(power);
    }

    public void setShooterPower(double power) {
        leftShooterMotor.setPower(power);
        rightShooterMotor.setPower(power);
    }

    public void setShooterVelocity(double velocity) {
        //Ticks per second
        leftShooterMotor.setVelocity(velocity);
        rightShooterMotor.setVelocity(velocity);
    }
    /*
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     */
//    public boolean isRightColor(SpikeColor colorWeWant) {
//        int blueThreshold = 4200;
//        int redThreshold = 980;
//        RGBAcolors colors = getSensorColors();
//
//        if (colorWeWant == SpikeColor.BLUE) {
//              return colors.getBlue() >= blueThreshold || colors.getRed() <= redThreshold;
//        }
//
//        if (colorWeWant == SpikeColor.RED) {
//            return colors.getRed() >= redThreshold || colors.getBlue() <= blueThreshold;
//        }
//
//        return false;
//    }

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