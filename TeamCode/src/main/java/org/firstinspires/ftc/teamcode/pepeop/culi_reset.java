package org.firstinspires.ftc.teamcode.pepeop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name ="culi_reset")
public class culi_reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        colectarev2 colectare= new colectarev2(hardwareMap);        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        while(opModeIsActive()){
       colectare.culisanta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       colectare.culisanta.setPower(-1);
//        colectare.transfer(j);
//            colectare.agatare();
//            colectare.extindere.setTargetPosition(i);
//            colectare.extindere.setPower(1);
//            colectare.extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("culi",colectare.culisanta.getCurrentPosition());
            telemetry.addData("capac",colectare.capac.getPosition());
            telemetry.addData("extindere",colectare.extindere.getCurrentPosition());
            telemetry.addData("grip_stng",colectare.griper_stanga.getPosition());
            telemetry.addData("grip_drpt",colectare.griper_dreapta.getPosition());
            telemetry.addData("perie_pozitie",colectare.perie_pozitie.getPosition());
            telemetry.addData("trands_dreapta",colectare.transfer_dreapta.getPosition());
            telemetry.addData("trands_stanga",colectare.transfer_stanga.getPosition());
            telemetry.addData("rotatie",colectare.rotite.getPosition());
            telemetry.addData("rotatie_grip",colectare.griper_rotatie.getPosition());
            telemetry.addData("senzor_cupa",colectare.distanta_cupa.getDistance(DistanceUnit.CM));
            telemetry.addData("extindere",colectare.agatare.getCurrentPosition());

            telemetry.update();
        }
//
    }}

