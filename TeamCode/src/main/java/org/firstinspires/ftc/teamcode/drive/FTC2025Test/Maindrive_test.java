package org.firstinspires.ftc.teamcode.drive.FTC2025Test;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//TODO: find drive motor Direction and setting IMU position  ->  DONE

//TODO: install entire servo and tuning

@TeleOp (name = "Main drive test", group = "2024-2025 Test OP")

public class Maindrive_test extends LinearOpMode {

    //pid settings
    private PIDController controller;

    public static double p = 0.04, i = 0, d = 0.001;

    public static double f = 0.001;

    public static int arm_target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    private DcMotorEx AL;
    private DcMotorEx AR;


    @Override
    public void runOpMode() throws InterruptedException {

        //pid setup
        controller = new PIDController(p,i,d);

        AL = hardwareMap.get(DcMotorEx.class, "AL");
        AR = hardwareMap.get(DcMotorEx.class, "AR");
        AR.setDirection(DcMotorSimple.Direction.REVERSE);

        //declear motor

        DcMotor FrontLeftMotor = hardwareMap.dcMotor.get("FL");
        DcMotor FrontRightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor BackLeftMotor = hardwareMap.dcMotor.get("BL");
        DcMotor BackRightMotor = hardwareMap.dcMotor.get("BR");

        //motor reverse
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMU settings
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        /*
        DcMotor ArmLeft = hardwareMap.dcMotor.get("AL");
        DcMotor ArmRight = hardwareMap.dcMotor.get("AR");*/

        Servo Vertical_Grip = hardwareMap.servo.get("V_Grip");   //vertical grip
        Servo Vertical_Grip_Wrist = hardwareMap.servo.get("V_Wrist");   //vertical grip wrist

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        imu.initialize(parameters);

        waitForStart();

        /*  //init for motors - WARNING, DO NOT ACTIVATE!
        AL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        //setting default var
        int arm_target = 0;

        boolean V_Grip_status = false;   //true is button pressed, true = grip closed
        boolean H_Grip_status = false;   //too

        boolean first_count = false;

        double V_Grip_OPEN = 0.3;
        double V_Grip_CLOSE = 0;

        //TODO: find Horizon Griper value
        double G_Grip_CLOSE = 0;

        int Low_basket = 2000;
        int High_basket = 4200;

        /*Gamepad.LedEffect sample_RED = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 100)
                .setRepeating(true)
                .build();

        Gamepad.LedEffect sample_BLUE = new Gamepad.LedEffect.Builder()
                .addStep(0, 0, 1, 100)
                .setRepeating(true)
                .build();

        Gamepad.LedEffect sample_YELLOW = new Gamepad.LedEffect.Builder()
                .addStep(1, 1, 0, 100)
                .setRepeating(true)
                .build();*/


        while (opModeIsActive()) {

            if (isStopRequested()) return;  //check stop button pushed?

            if (first_count) {
                //write code


                first_count = false;
            }

            //pid calculate
            controller.setPID(p,i,d);

            int ArmPos = AL.getCurrentPosition();
            double pid = controller.calculate(ArmPos, arm_target);
            double ff = Math.cos(Math.toRadians(arm_target / ticks_in_degree))*f;

            double ArmPower = pid + ff;

            AL.setPower(ArmPower);
            AR.setPower(ArmPower);

            //
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            //
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //
            double y = -gamepad1.left_stick_y;  //y value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double slow = 1 - (0.8 * gamepad1.right_trigger);   //slow mechanism, change 0.8 to slow leveling

            //
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX - rx) / denominator) * slow;
            double backLeftPower = ((rotY - rotX - rx) / denominator) * slow;
            double frontRightPower = ((rotY - rotX + rx) / denominator) * slow;
            double backRightPower = ((rotY + rotX + rx) / denominator) * slow;

            FrontLeftMotor.setPower(frontLeftPower);
            BackLeftMotor.setPower(backLeftPower);
            FrontRightMotor.setPower(frontRightPower);
            BackRightMotor.setPower(backRightPower);


            //continue main coading

            if (gamepad1.left_bumper) {

            } else {

            }

            if (rising_edge(currentGamepad1.a, previousGamepad1.a)) {

            }







            //telemetry settings
            telemetry.addData("ArmPos ", ArmPos);
            telemetry.addData("Target Pos ", arm_target);
            telemetry.addData("V_Grip Pos ", Vertical_Grip);
            telemetry.addData("V_Grip_Wrist ", Vertical_Grip_Wrist);
            telemetry.update();  //update telemetry, end of line
        }

    }

    private boolean rising_edge(boolean currentButtonState, boolean previousButtonState) {
        return currentButtonState && !previousButtonState;
    }
}
