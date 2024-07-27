package org.firstinspires.ftc.teamcode.pepeop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="progi")
@Config
public class programator extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        colectarev2 colectare= new colectarev2(hardwareMap);        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        double i=colectare.stack5;
        int j=0;
//        double j=colectare.griper_stanga_open ;

        waitForStart();
        while(opModeIsActive()&& !isStopRequested()){

            if(gamepad1.dpad_up)i+=0.0005;
            if(gamepad1.dpad_right)j+=10;
            if(gamepad1.dpad_down)i-=0.0005;
            if(gamepad1.dpad_left)j-=10;
//            colectare.extindere.setTargetPosition(i);
//            colectare.extindere.setPower(1);
//            colectare.extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            colectare.agatare.setTargetPosition(j);
//            colectare.agatare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            colectare.agatare.setPower(1);

//            colectare.perie_pozitie.setPosition(i);
//            if(gamepad1.right_trigger!=0)colectare.perie.setPower(-1);

colectare.perie_pozitie.setPosition(i);
colectare.perie.setPower(-1);
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
                        telemetry.addData("agatare",colectare.agatare.getCurrentPosition());
                        telemetry.addData("j",j);
                        telemetry.addData("drone",colectare.drone.getPosition());

            telemetry.update();
        }
//
    }
}
