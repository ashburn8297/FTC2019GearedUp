package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.robotBaseAuto.COUNTS_PER_INCH;

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

        double speed=.2;
        int newLeftTarget;
        int newRightTarget;

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine new target position, and pass to motor controller
        newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (20 * COUNTS_PER_INCH);
        newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (20 * COUNTS_PER_INCH);
        robot.leftDrive.setTargetPosition(newLeftTarget);
        robot.rightDrive.setTargetPosition(newRightTarget);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.leftDrive.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() && (runtime.seconds() < 4.0) && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            double curPos = Math.abs(robot.leftDrive.getCurrentPosition());
            double toGo = Math.abs(newLeftTarget) - curPos;
            //0 to 560*50
            double pct = toGo/newLeftTarget;
            robot.leftDrive.setPower(Math.abs(speed * pct) + .1);
            robot.rightDrive.setPower(Math.abs(speed * pct) + .1);
            telemetry.addData("Reached","Speed Code");
            telemetry.addData("Percent", pct);
            telemetry.addData("Target", newLeftTarget);
            telemetry.addData("Power", robot.leftDrive.getPower());
            telemetry.update();
            sleep(500);
        }


        sleep (5000);
    }


}
