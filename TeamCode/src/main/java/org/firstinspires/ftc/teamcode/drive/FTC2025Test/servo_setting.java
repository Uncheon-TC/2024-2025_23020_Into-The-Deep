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

        Servo V_wristL = hardwareMap.servo.get("V_wristL");
        Servo V_grip = hardwareMap.servo.get("V_grip");

        double wrist = 0;
        double grip = 0;

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
                wrist = wrist + 0.01;
                V_wristL.setPosition(wrist);
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                wrist = wrist - 0.01;
                V_wristL.setPosition(wrist);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                wrist = wrist + 0.1;
                V_wristL.setPosition(wrist);
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                wrist = wrist - 0.1;
                V_wristL.setPosition(wrist);
            }



            telemetry.addData("grip", wrist);
            telemetry.addData("angle", grip);
            telemetry.update();
        }
    }
}
