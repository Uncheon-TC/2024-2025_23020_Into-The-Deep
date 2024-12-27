package org.firstinspires.ftc.teamcode.drive.FTC2025Test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: find optimal servo position - gripper and gripper angle(90 degree and 0 degree)

@TeleOp(name = "servo_setting", group = "2024-2025 Test OP")

public class servo_setting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo gripper = hardwareMap.servo.get("Grip");
        Servo Gangle = hardwareMap.servo.get("GAngle");

        double grip_target = 0;
        double gripAngle_target = 0;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            //
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.a && !previousGamepad1.a) {
                grip_target = grip_target + 0.1;
                gripper.setPosition(grip_target);
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                grip_target = grip_target - 0.1;
                gripper.setPosition(grip_target);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                gripAngle_target = gripAngle_target + 0.1;
                Gangle.setPosition(gripAngle_target);
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                gripAngle_target = gripAngle_target - 0.1;
                Gangle.setPosition(gripAngle_target);
            }

            telemetry.addData("grip", grip_target);
            telemetry.addData("angle", gripAngle_target);
            telemetry.update();
        }
    }
}
