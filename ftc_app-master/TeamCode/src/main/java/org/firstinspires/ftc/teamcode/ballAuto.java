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


@Autonomous(name = "BallAuto SAFE")
//@Disabled
public class ballAuto extends LinearOpMode {
    robotBaseAuto robot = new robotBaseAuto();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.traverse.setPosition(midTraverseRight);
        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        runtime.reset();
        while (robot.navxMicro.isCalibrating())  {
            telemetry.addData("Calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(100);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();

        telemetry.update();

        waitForStart();
        runtime.reset();

        //Lower Lift
        if (opModeIsActive()){
            robot.ADM.setTargetPosition((int) (robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev) - 100);
            robot.ADM.setPower(.95);
            telemetry.addData("Lift Encoder Value", robot.ADM.getCurrentPosition());
        }

        sleep(3000);
        robot.ADM.setPower(.05); //To stop jittering

        //Slide over
        if (opModeIsActive()){
            robot.traverse.setPosition(robot.maxTraverse);
            sleep(2000);
            robot.encoderDriveStraight(26, 5.0, .24, opModeIsActive(), runtime);
        }
        sleep(1000);
    }
}
