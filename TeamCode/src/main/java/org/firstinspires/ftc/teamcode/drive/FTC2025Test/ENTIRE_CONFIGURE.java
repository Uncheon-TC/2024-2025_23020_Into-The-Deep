package org.firstinspires.ftc.teamcode.drive.FTC2025Test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config

@TeleOp
public class ENTIRE_CONFIGURE extends OpMode{
    private PIDController controller;

    public final double p = 0.02, i = 0, d = 0.0005;
    public final double f = 0.001;

    public static int arm_target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    public static double POS_V_grip = 0.5;
    public static double POS_V_wristL = 0.5;
    public static double POS_H_wristL = 0.5;
    public static double POS_H_wristR = 0.5;
    public static double POS_H_grip = 0.5;
    public static double POS_H_length = 0.5;
    public static double POS_H_angle = 0.5;
    //public static double POS_H_angleR = 0.5;


    //private Servo V_wristR;
    private Servo V_wristL;
    private Servo H_length;
    private Servo H_wristL;
    private Servo H_wristR;
    private Servo H_angleL;
    private Servo H_angleR;
    private Servo H_grip;
    private Servo V_grip;


    private DcMotorEx AL;
    private DcMotorEx AR;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        AL = hardwareMap.get(DcMotorEx.class, "AL");
        AR = hardwareMap.get(DcMotorEx.class, "AR");

        AR.setDirection(DcMotorSimple.Direction.REVERSE);
//
        //Servo V_wristR = hardwareMap.servo.get("V_wristR"); //Bucket Wrist right Servo
        V_wristL = hardwareMap.servo.get("V_wristL"); //Bucket Wrist left Servo
        H_length = hardwareMap.servo.get("H_length"); //Slide right Servo
        H_wristR = hardwareMap.servo.get("H_wristR"); // Ground Gripper right Servo
        H_wristL = hardwareMap.servo.get("H_wristL"); // Ground Gripper Left Servo
        H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
        H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo
        H_grip = hardwareMap.servo.get("H_grip");
        V_grip = hardwareMap.servo.get("V_grip"); //vertical grip wrist

        H_wristL.setDirection(Servo.Direction.REVERSE);

        H_angleL.setDirection(Servo.Direction.REVERSE);

        POS_H_angle = H_angleL.getPosition();

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = AL.getCurrentPosition();
        double pid = controller.calculate(armPos, arm_target);
        double ff = Math.cos(Math.toRadians(arm_target / ticks_in_degree)) * f;



        double power = pid + ff;

        AL.setPower(power);
        AR.setPower(power);


        H_wristL.setPosition(POS_H_wristL);
        H_wristR.setPosition(POS_H_wristR);

        V_grip.setPosition(POS_V_grip);
        H_grip.setPosition(POS_H_grip);

        V_wristL.setPosition(POS_V_wristL);

        H_length.setPosition(POS_H_length);

        H_angleL.setPosition(POS_H_angle);
        H_angleR.setPosition(POS_H_angle);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", arm_target);
        telemetry.update();
    }
}
