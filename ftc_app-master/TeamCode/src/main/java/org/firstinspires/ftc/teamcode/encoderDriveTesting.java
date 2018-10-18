package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.firstinspires.ftc.teamcode.robotBase;

@TeleOp(name = "Encoder Drive Test")
//@Disabled
public class encoderDriveTesting extends TunableLinearOpMode {

    robotBase robot = new robotBase();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Initial",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        * REVISIONS
        * in the robotBase class, the encoderDriveStraight and encoderDriveTurn methods were edited.
        * The loop checks if the motors are busy, and the change from && to || should alleviate some turning inconsistencies.
         */
        //@TODO verify the accuracy of the methods. Change variables in robotBase.java if incorrect
        // negative is right, and negative is forward

            robot.encoderDriveTurn(-90, 4.0, opModeIsActive(), runtime);
            robot.encoderDriveStraight(-24, 2.0, opModeIsActive(), runtime);
            robot.encoderDriveTurn(180, 4.0, opModeIsActive(), runtime);
            robot.encoderDriveStraight(-48, 2.0, opModeIsActive(), runtime);
            robot.encoderDriveTurn(180, 4.0, opModeIsActive(), runtime);
            robot.encoderDriveStraight(24, 2.0, opModeIsActive(), runtime);
            robot.encoderDriveTurn(90, 4.0, opModeIsActive(), runtime);

    }

}
