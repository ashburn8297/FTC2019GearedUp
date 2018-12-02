package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.teamcode.robotBase.midTraverseRight;


@Autonomous(name = "BallAuto SAFE")
//@Disabled
public class ballAuto extends LinearOpMode {
    robotBase robot = new robotBase();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.traverse.setPosition(midTraverseRight);
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();
        runtime.reset();
        telemetry.update();

        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.inVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Raise ADM, and drop from lander

        robot.ADM.setTargetPosition((int)(robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev)-100); //tuner
        robot.ADM.setPower(.5);
        telemetry.addData("Lift Encoder Value", robot.ADM.getCurrentPosition());
        sleep(5000);

        //Move ADM to the side, move marker, keep box raised
        robot.traverse.setPosition(robot.maxTraverse);
        sleep(1000);
        robot.marker.setPosition(robot.markerMid);
        sleep(500);
        robot.intakePitch.setPosition(robot.boxFlat);
        robot.intake.setPower(-1);
        sleep(500);
        robot.marker.setPosition(robot.markerIn);
        sleep(500);

        robot.encoderDriveStraight(18, 3, opModeIsActive(), runtime);
        robot.intake.setPower(0);
        sleep(2000);

        stop();
    }
}
