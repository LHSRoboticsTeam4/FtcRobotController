package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Instead of each Op Mode class redefining the robot's hardware resources within its implementation,
 * This RobotHardware class has a given robot's component resources defined and set up all in one place.
 * It also has convenience methods like driveRobot(), setArmPower(), setHandPosition(), etc. that work for that robot.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */
public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define all the HardwareDevices (Motors, Servos, etc.). Make them private so they can't be accessed externally.
    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftRearWheel;
    private DcMotor rightRearWheel;

    private Servo launchServo;

    // Define other HardwareDevices as needed.
    private DistanceSensor rightPropSensor;
    private DistanceSensor leftPropSensor;
    private ColorSensor colorSensor;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DEFAULT_WHEEL_MOTOR_SPEED = .4;
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    static final double SENSOR_DISTANCE_OUT_OF_RANGE = 20 ;

    /**
     * The one and only constructor requires a reference to an OpMode.
     *
     * @param opmode
     */
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initWheelMotors();
        initSensors();
        initServo();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Initialize all the wheel motors.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    private void initWheelMotors()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontWheel  = myOpMode.hardwareMap.get(DcMotor.class, "LFront");
        rightFrontWheel = myOpMode.hardwareMap.get(DcMotor.class, "RFront");
        leftRearWheel = myOpMode.hardwareMap.get(DcMotor.class, "LRear");
        rightRearWheel = myOpMode.hardwareMap.get(DcMotor.class, "RRear");

        // To drive forward, most robots need the motors on one side to be reversed, because the axles point in opposite directions.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheel.setDirection(DcMotor.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheel.setDirection(DcMotor.Direction.REVERSE);

        // Reset wheel encoders
        setRunModeForAllWheels(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set wheel motors to not resist turning when motor is stopped.
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void initSensors() {
        rightPropSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "rightSensor");
        leftPropSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "leftSensor");
        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    private void initServo() {
        launchServo = myOpMode.hardwareMap.get(Servo.class, "DroneLaunch");
    }

    /**
     * Drive robot to the targeted position designated by the passed leftInches and
     * rightInches, at the power specified by speed.
     * @param leftInches
     * @param rightInches
     * @param speed
     */
    public void autoDriveRobot(int leftInches, int rightInches, double speed) {
        /**
         * This shouldn't be needed, but without it, sometimes the left and right sides go in
         * opposite directions even when the leftInches and rightInches are the same values!
         */
        setRunModeForAllWheels(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int leftInchesToCPI = (int) (leftInches * COUNTS_PER_INCH);
        int rightInchesToCPI = (int) (rightInches * COUNTS_PER_INCH);

        int leftFrontTarget = leftFrontWheel.getCurrentPosition() + leftInchesToCPI;
        int leftRearTarget = leftFrontWheel.getCurrentPosition() + leftInchesToCPI;
        int rightFrontTarget = leftFrontWheel.getCurrentPosition() + rightInchesToCPI;
        int rightRearTarget = leftFrontWheel.getCurrentPosition() + rightInchesToCPI;

        leftFrontWheel.setTargetPosition(leftFrontTarget);
        leftRearWheel.setTargetPosition(leftRearTarget);
        rightFrontWheel.setTargetPosition(rightFrontTarget);
        rightRearWheel.setTargetPosition(rightRearTarget);

        setRunModeForAllWheels(DcMotor.RunMode.RUN_TO_POSITION);

        //Set up telemetry
        myOpMode.telemetry.setAutoClear(false);
        myOpMode.telemetry.addData("Heading", "Current Wheel Positions");
        Telemetry.Item leftFrontWheelItem = myOpMode.telemetry.addData("LF Wheel", leftFrontWheel.getCurrentPosition());
        Telemetry.Item leftRearWheelItem = myOpMode.telemetry.addData("LR Wheel", leftRearWheel.getCurrentPosition());
        Telemetry.Item rightFrontWheelItem = myOpMode.telemetry.addData("RF Wheel", rightFrontWheel.getCurrentPosition());
        Telemetry.Item rightRearWheelItem = myOpMode.telemetry.addData("RR Wheel", rightRearWheel.getCurrentPosition());
        myOpMode.telemetry.update();

        setPowerAllWheels(Math.abs(speed));

        // Update telemetry for as long as the wheel motors isBusy().
        while (leftFrontWheel.isBusy() && leftRearWheel.isBusy() && rightFrontWheel.isBusy() && rightRearWheel.isBusy()) {
            leftFrontWheelItem.setValue(leftFrontWheel.getCurrentPosition());
            leftRearWheelItem.setValue(leftRearWheel.getCurrentPosition());
            rightFrontWheelItem.setValue(rightFrontWheel.getCurrentPosition());
            rightRearWheelItem.setValue(rightRearWheel.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        //Robot has RUN_TO_POSITION.
        setPowerAllWheels(0); //Whoa.
        myOpMode.telemetry.setAutoClear(true);
    }

    /**
     * autoDriveRobot using DEFAULT_WHEEL_MOTOR_SPEED.
     * @param leftInches
     * @param rightInches
     */
    public void autoDriveRobot(int leftInches, int rightInches) {
        autoDriveRobot(leftInches, rightInches, DEFAULT_WHEEL_MOTOR_SPEED);
    }

    public double getRightPropDistanceInCM() {
        return rightPropSensor.getDistance(DistanceUnit.CM);
    }

    public double getLeftPropDistanceInCM() {
        return leftPropSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Set the RunMode for all wheel motors to the passed runMode.
     * @param runMode
     */
    public void setRunModeForAllWheels(DcMotor.RunMode runMode) {
        leftFrontWheel.setMode(runMode);
        leftRearWheel.setMode(runMode);
        rightFrontWheel.setMode(runMode);
        rightRearWheel.setMode(runMode);
    }

    /**
     * Set the Power for all wheels to the passed speed.
     * @param speed
     */
    public void setPowerAllWheels(double speed) {
        leftFrontWheel.setPower(speed);
        leftRearWheel.setPower(speed);
        rightFrontWheel.setPower(speed);
        rightRearWheel.setPower(speed);
    }

    /**
     * Drive robot according to passed stick inputs.
     * @param stick1X Value from stick 1's X axis
     * @param stick1Y Value from stick 1's Y axis
     * @param stick2X Value from stick 2's X axis
     */
    public void manuallyDriveRobot(double stick1X, double stick1Y, double stick2X) {
        double vectorLength = Math.hypot(stick1X, stick1Y);
        double robotAngle = Math.atan2(stick1Y, -stick1X) - Math.PI / 4;
        double rightXscale = stick2X * .5;
        final double rightFrontVelocity = vectorLength * Math.cos(robotAngle) + rightXscale;
        final double leftFrontVelocity = vectorLength * Math.sin(robotAngle) - rightXscale;
        final double rightRearVelocity = vectorLength * Math.sin(robotAngle) + rightXscale;
        final double leftRearVelocity = vectorLength * Math.cos(robotAngle) - rightXscale;
        setRunModeForAllWheels(DcMotor.RunMode.RUN_USING_ENCODER);
        // Use existing method to drive both wheels.
        setDrivePower(leftFrontVelocity, rightFrontVelocity, leftRearVelocity, rightRearVelocity);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        // Output the values to the motor drives.
        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftRearWheel.setPower(leftRearPower);
        rightRearWheel.setPower(rightRearPower);
    }

    /**
     * Move XYZ servo so that drone is released.
     */
    public void releaseDrone() {
        //Move whichever servo(?).
        launchServo.setPosition(0.3);
    }

    public void resetDrone() {
        launchServo.setPosition(0);
    }

    public void turnToSpike(int positionNumber) {
        if ( positionNumber == 2 ) {
            autoDriveRobot(10, 10); // 2 is in the middle, so we leave the pixel alone and backup.
        }
        else if (positionNumber == 1) {
            autoDriveRobot (28, -28) ; // turn to the left
            autoDriveRobot (-5, -5); // go forward to place pixel
            autoDriveRobot(10, 10); // backup to clear pixel

        }
        else {
            autoDriveRobot (-28, 28) ; // turn to the right
            autoDriveRobot (-5, -5); // go forward to place pixel
            autoDriveRobot(10, 10); // backup to clear pixel
        }
    }
    /**
     * Drive robot until the passed color is detected.
     * @param color
     * @param colorThreshold
     */
    public void driveToSpike(SpikeColor color, int colorThreshold,
                             double approachSpeed) {
        RGBAcolors colors;

        setRunModeForAllWheels(DcMotor.RunMode.RUN_USING_ENCODER);
        setPowerAllWheels(approachSpeed);

        while (myOpMode.opModeIsActive()) {
            colors = getSensorColors();
            if (color == SpikeColor.RED && colors.getRed() > colorThreshold) {
                break;
            }
            else if (colors.getBlue() > colorThreshold) {
                break;
            }
        }

        setPowerAllWheels(0);
    }
    /**
     * Return all color values from Color Sensor.
     * @return RGBAcolor
     */
    public RGBAcolors getSensorColors() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int alpha = colorSensor.alpha();

        return new RGBAcolors(red, green, blue, alpha);
    }
    /**
     * Determine which position (1, 2, or 3) the sensor detects an object (such as a cube) is in.
     * In this example, if neither the left or right sensors detect an object,
     * the position is 2. If the left sensor detects an object, the position is 1. Lastly,
     * if the right sensor detects an object, the position is 3.
     *
     * @return int positionNumber
     */
    public int getSpikeObjectPosition() {
        double leftSensorDistance = getLeftSensorDistanceInCM();
        double rightSensorDistance = getRightSensorDistanceInCM();
        int positionNumber = 0;

        if (leftSensorDistance >= SENSOR_DISTANCE_OUT_OF_RANGE && rightSensorDistance >= SENSOR_DISTANCE_OUT_OF_RANGE) {
            myOpMode.telemetry.addData("NOT DETECTED", "Object not detected by any sensor!");
            positionNumber = 2;
        }
        else if (leftSensorDistance <= SENSOR_DISTANCE_OUT_OF_RANGE) {
            myOpMode.telemetry.addData("DETECTED LEFT", "Object distance is %.0f CM", leftSensorDistance);
            positionNumber = 1;
        }
        else {
            myOpMode.telemetry.addData("DETECTED RIGHT", "Object distance is %.0f CM", rightSensorDistance);
            positionNumber = 3;
        }
        myOpMode.telemetry.update();

        return positionNumber;
    }
    /**
     * Return distance detected by Left Sensor in CM.
     * @return distance in CM
     */
    public double getLeftSensorDistanceInCM() {
        return leftPropSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Return distance detected by Right Sensor in CM.
     * @return distance in CM
     */
    public double getRightSensorDistanceInCM() {
        return rightPropSensor.getDistance(DistanceUnit.CM);
    }
}

