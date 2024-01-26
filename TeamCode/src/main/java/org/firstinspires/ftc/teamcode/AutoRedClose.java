package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoRedClose", group = "")
public class AutoRedClose extends LinearOpMode {
    private RobotHardware robot;
    @Override
    public void runOpMode() {

        //RobotHardware robot = new RobotHardware(this);
        // robot.init();
        robot = new RobotHardware(this);
        robot.init();

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

                // Set these variables to a initial distance, you will need to change this number
                int driveLeftInches = -25;
                int driveRightInches = -25;
                robot.autoDriveRobot(driveLeftInches, driveRightInches);
                robot.driveToSpike(SpikeColor.RED,100,-.1 );
                int positionNumber = robot.getSpikeObjectPosition();
                robot.turnToSpike(positionNumber);
        }
    }


}   // end class

