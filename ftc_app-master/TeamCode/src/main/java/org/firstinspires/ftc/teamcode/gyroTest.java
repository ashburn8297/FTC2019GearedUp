package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.robotBaseAuto.COUNTS_PER_INCH;

@Autonomous(name = "Gyro Test")
public class gyroTest extends LinearOpMode {
    robotBaseAuto robot = new robotBaseAuto();
    private ElapsedTime runtime = new ElapsedTime();
    private boolean rateFound = false;

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

        waitForStart();
        runtime.reset();
        double previousRate = 0.0;
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(!rateFound && runtime.seconds() < 1.5) {
            robot.leftDrive.setPower(-.2);
            robot.rightDrive.setPower(-.2);
            AngularVelocity rates = robot.gyro.getAngularVelocity(AngleUnit.DEGREES);
            double rate = rates.xRotationRate;
            if((Math.abs(rate - previousRate)> 9) && (runtime.seconds() > .75)) {
                telemetry.addData("HIT", rate);
                telemetry.update();
                robot.brake();
                rateFound = true;
            }
            previousRate = rate;
        }
        sleep(10000);
    }


}
