package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.teamcode.robotBase.midTraverseRight;


@Autonomous(name = "CubeAuto SAFE")
//@Disabled
public class cubeAuto extends TunableLinearOpMode {
    robotBase robot = new robotBase();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.traverse.setPosition(midTraverseRight);
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to");
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

        //Move lift left
        robot.traverse.setPosition(robot.maxTraverse);
        sleep(1000);

        robot.marker.setPosition(robot.markerMid);
        sleep(500);
        robot.intakePitch.setPosition(robot.boxFlat);
        robot.intake.setPower(-1);
        sleep(500);
        robot.marker.setPosition(robot.markerIn);
        sleep(500);

        robot.encoderDriveStraight(55, 5, opModeIsActive(), runtime);
        robot.intake.setPower(0);
        sleep(1000);
        robot.turnByGyro(-45, .2, opModeIsActive());
        sleep(500);
        robot.brake();

        robot.marker.setPosition(robot.markerOut);
        sleep(1000);

        stop();
    }
}
