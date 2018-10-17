package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.firstinspires.ftc.teamcode.robotBase;


@Autonomous(name="Auto Gyro Drive Test")
public class autoGyroDriveTesting extends TunableLinearOpMode{

    robotBase robot = new robotBase();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.brake();

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
            telemetry.update();
        }

        robot.gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        // Each turn should compound on the one before it, so a -45 and 45 degree turn net 0.
        // All units are inches and degrees unless otherwise specified.
        robot.gyroDrive(robot.DRIVE_SPEED, 48.0, 0.0, opModeIsActive());         // Drive FWD 48 inches
        robot.gyroTurn( robot.TURN_SPEED, -45.0, opModeIsActive());                       // Turn  CCW to -45 Degrees
        robot.gyroHold( robot.TURN_SPEED, -45.0, 0.5, opModeIsActive());        // Hold -45 Deg heading for a 1/2 second
        robot.gyroDrive(robot.DRIVE_SPEED, 12.0, -45.0, opModeIsActive());       // Drive FWD 12 inches at 45 degrees
        robot.gyroTurn( robot.TURN_SPEED,  45.0, opModeIsActive());                       // Turn  CW  to  45 Degrees
        robot.gyroHold( robot.TURN_SPEED,  45.0, 0.5,  opModeIsActive());        // Hold  45 Deg heading for a 1/2- second
        robot.gyroTurn( robot.TURN_SPEED,   0.0, opModeIsActive());                       // Turn  CW  to   0 Degrees
        robot.gyroHold( robot.TURN_SPEED,   0.0, 1.0, opModeIsActive());         // Hold  0 Deg heading for a 1 second
        robot.gyroDrive(robot.DRIVE_SPEED,-48.0, 0.0, opModeIsActive());         // Drive REV 48 inches

        telemetry.addData("Path", "Complete in %d", runtime);
        telemetry.update();
    }
}
