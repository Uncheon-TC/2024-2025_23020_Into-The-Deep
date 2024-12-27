package org.firstinspires.ftc.teamcode.drive.FTC2025Test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: find optimal servo position - gripper and gripper angle(90 degree and 0 degree)

@TeleOp(name = "angle_servo_test", group = "2024-2025 Test OP")

public class angle_servo_test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo L = hardwareMap.servo.get("L");
        Servo R = hardwareMap.servo.get("R");

        double L_target = 0;
        double R_target = 0;

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
                L_target = L_target + 0.1;
                L.setPosition(L_target);
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                L_target = L_target - 0.1;
                L.setPosition(L_target);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                R_target = R_target + 0.1;
                R.setPosition(R_target);
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                R_target = R_target - 0.1;
                R.setPosition(R_target);
            }

            telemetry.addData("L", L_target);
            telemetry.addData("R", R_target);
            telemetry.update();
        }
    }
}
