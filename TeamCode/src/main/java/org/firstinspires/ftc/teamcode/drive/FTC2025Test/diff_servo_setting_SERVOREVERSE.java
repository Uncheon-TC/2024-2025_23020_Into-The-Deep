package org.firstinspires.ftc.teamcode.drive.FTC2025Test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: find optimal servo position - gripper and gripper angle(90 degree and 0 degree)


@Config

@TeleOp(name = "DIFF_servo_setting_SERVOREVERSE", group = "2024-2025 Test OP")

public class diff_servo_setting_SERVOREVERSE extends LinearOpMode {

    private Servo H_wristL;
    private Servo H_wristR;

    public static double interval = 0.05;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        H_wristL = hardwareMap.servo.get("H_wristL");
        H_wristR = hardwareMap.servo.get("H_wristR");

        H_wristL.setDirection(Servo.Direction.REVERSE);

        double wristL = 0.5;
        double wristR = 0.5;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        wrist_control(wristL, wristR);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            //
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            /*

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
            */

            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                wristL = wristL + interval;
                wristR = wristR - interval;

                if (wristL > 1) {
                    wristL = 1;
                }

                if (wristR < 0) {
                    wristR = 0;
                }

                wrist_control(wristL, wristR);
            }

            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                wristL = wristL - interval;
                wristR = wristR + interval;

                if (wristL < 0) {
                    wristL = 0;
                }

                if (wristR > 1) {
                    wristR = 1;
                }

                wrist_control(wristL, wristR);
            }



            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                wristL = wristL + interval;
                wristR = wristR + interval;

                if (wristL > 1) {
                    wristL = 1;
                }

                if (wristR > 1) {
                    wristR = 1;
                }

                wrist_control(wristL, wristR);

            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                wristL = wristL - interval;
                wristR = wristR - interval;

                if (wristL < 0) {
                    wristL = 0;
                }

                if (wristR < 0) {
                    wristR = 0;
                }

                wrist_control(wristL, wristR);
            }



            telemetry.addData("wristL ", wristL);
            telemetry.addData("wristR ", wristR);
            telemetry.update();
        }


    }

    private void wrist_control(double L, double R) {
        H_wristL.setPosition(L);
        H_wristR.setPosition(R);
    }

}

