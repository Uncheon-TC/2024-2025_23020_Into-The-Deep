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

    public static double interval = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        H_wristL = hardwareMap.servo.get("H_wristL");
        H_wristR = hardwareMap.servo.get("H_wristR");

        H_wristL.setDirection(Servo.Direction.REVERSE);

        double wrist = 0;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        wrist_Hcontrol(0.5);

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
                wrist = wrist + interval;
                wrist_Hcontrol(wrist);
            }

            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                wrist = wrist - interval;
                wrist_Hcontrol(wrist);
            }



            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                wrist = wrist + interval;
                wrist_Vcontrol(wrist);

            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                wrist = wrist - interval;
                wrist_Vcontrol(wrist);
            }



            telemetry.addData("POS ", wrist);

            telemetry.update();
        }


    }

    private void wrist_Vcontrol(double pos) {
        H_wristR.setPosition(pos);
        H_wristL.setPosition(pos);
    }

    private void wrist_Hcontrol(double pos) {
        H_wristR.setPosition(pos);
        H_wristL.setPosition(1-pos);
    }

}

