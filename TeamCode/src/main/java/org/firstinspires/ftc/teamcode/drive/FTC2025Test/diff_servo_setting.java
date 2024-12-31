package org.firstinspires.ftc.teamcode.drive.FTC2025Test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: find optimal servo position - gripper and gripper angle(90 degree and 0 degree)

@TeleOp(name = "DIFF_servo_setting", group = "2024-2025 Test OP")

public class diff_servo_setting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo H_wristL = hardwareMap.servo.get("V_wristL");
        Servo H_wristR = hardwareMap.servo.get("V_wristR");

        double wristL = 0;
        double wristR = 0;

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
                wristL = wristL + 0.01;
                H_wristL.setPosition(wristL);
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                wristL = wristL - 0.01;
                H_wristL.setPosition(wristL);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                wristL = wristL + 0.1;
                H_wristL.setPosition(wristL);
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                wristL = wristL - 0.1;
                H_wristL.setPosition(wristL);
            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                wristR = wristR + 0.01;
                H_wristR.setPosition(wristR);
            }

            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                wristR = wristR - 0.01;
                H_wristR.setPosition(wristR);
            }

            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                wristR = wristR + 0.1;
                H_wristR.setPosition(wristR);
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                wristR = wristR - 0.1;
                H_wristR.setPosition(wristR);
            }



            telemetry.addData("L ", wristL);
            telemetry.addData("R ", wristR);
            telemetry.update();
        }
    }
}
