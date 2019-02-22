package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;

@Autonomous(name = "Gyro Test")
public class gyroTest extends LinearOpMode {
    robotBaseAuto robot = new robotBaseAuto();
    private ElapsedTime runtime = new ElapsedTime();

    double turn_speed = .15;
    public void runOpMode() {
        robot.init(hardwareMap);
        while (robot.navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        /*robot.turnByGyro(90, turn_speed, opModeIsActive(), 2.0, runtime);
        sleep(500);
        robot.turnByGyro(0, turn_speed, opModeIsActive(), 2.0, runtime);
        sleep(500);
        while(opModeIsActive()){
            telemetry.addData("Angle:",  robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }*/
        while(Math.abs(robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle) < 5){
            robot.encoderDriveStraight(4, 1.0, .1, opModeIsActive(), runtime);
        }
        sleep (5000);
    }


}
