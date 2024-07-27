package org.firstinspires.ftc.teamcode.pepeop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="micro")
public class micro extends LinearOpMode {
    Servo st,dr;
    @Override
    public void runOpMode() throws InterruptedException {
        colectarev2 colectare = new colectarev2(hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.triangle) {
                colectare.griper_stanga.setPosition(colectare.griper_stanga_close);
                colectare.griper_dreapta.setPosition(colectare.griper_dreapta_close);
            }
            if (gamepad1.square) {
                colectare.griper_stanga.setPosition(colectare.griper_stanga_open);
                colectare.griper_dreapta.setPosition(colectare.griper_dreapta_open);
            }
        }
    }}
