package org.firstinspires.ftc.teamcode.pepeop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(name="pepe_v2")
public class Teleop_v2 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        colectarev2 colectare = new colectarev2(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        colectare.tele_init();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime gametime=new ElapsedTime();
        ElapsedTime timerz = new ElapsedTime();
        ElapsedTime teto= new ElapsedTime();
        ElapsedTime controalere=new ElapsedTime();
        ElapsedTime periuta=new ElapsedTime();
        boolean perius=false;
        boolean controlor=false;
        boolean autos=false;
        int exti_hold=0;
        boolean colect=false;
        boolean automatik=false;
        boolean endgame=false;
        int schema=0;
        int agatatu =0;
        int endtindere=0;
        int gripers=3;
      boolean  culis_miscare=false;
        boolean mediu=false;
        boolean culi=false;
//        colectare.cupa_dist=true;
//drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive() && !isStopRequested())
        {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
//            if(gamepad2.touchpad)automatik=true;
//            if(gamepad2.share)automatik=false;
            if(gamepad2.left_trigger!=0){
                endgame=true;
            }
            if(gamepad2.options)endgame=false;
            if(!endgame){
            if(schema==1) {
                if (gamepad2.dpad_down){ colectare.culisanta_pozitie = 600; colectare.score(); schema=0;}
                if (gamepad2.dpad_left) {colectare.culisanta_pozitie = 1200; colectare.score(); schema=0;}
                if (gamepad2.dpad_up) {colectare.culisanta_pozitie = 1800;colectare.score(); schema=0;}
            }
            }

//                if(gamepad2.dpad_up)colectare.griper_rotatie.setPosition(colectare.rot_orizont);
//                if(gamepad2.dpad_down)colectare.griper_rotatie.setPosition(colectare.rot_orizont_invers);
//                if(gamepad2.dpad_right)colectare.griper_rotatie.setPosition(colectare.rot_drept);
//                if(gamepad2.x)colectare.griper_rotatie.setPosition(colectare.rot_drept_invers);
//                if(gamepad2.right_trigger!=0)colectare.griper_rotatie.setPosition(colectare.griper_rotatie.getPosition()+0.1);
//                if(gamepad2.left_trigger!=0)colectare.griper_rotatie.setPosition(colectare.griper_rotatie.getPosition()-0.1);

//                        if (gamepad1.touchpad) {
//                            if (gamepad1.dpad_up)
//                                colectare.griper_rotatie.setPosition(colectare.rot_orizont_invers);
//                            if (gamepad1.dpad_down)
//                                colectare.griper_rotatie.setPosition(colectare.rot_diag_dreapta);
//                            if (gamepad1.dpad_right)
//                                colectare.griper_rotatie.setPosition(colectare.rot_drept_invers);
//                            if (gamepad1.dpad_left)
//                                colectare.griper_rotatie.setPosition(colectare.rot_diag_stanga);
//                        } else {
//                            if (gamepad1.dpad_up)
//                                colectare.griper_rotatie.setPosition(colectare.rot_orizont);
//                            if (gamepad1.dpad_down)
//                                colectare.griper_rotatie.setPosition(colectare.rot_diag_dreapta);
//                            if (gamepad1.dpad_right)
//                                colectare.griper_rotatie.setPosition(colectare.rot_drept);
//                            if (gamepad1.dpad_left)
//                                colectare.griper_rotatie.setPosition(colectare.rot_diag_stanga);
//                    }
            if(controlor==false) {
                if (gamepad1.dpad_down){ controlor=true; controalere.reset(); gripers--;}
                if (gamepad1.dpad_up){controlor=true;controalere.reset(); gripers++;}
                if (gamepad1.dpad_left){ controlor=true; controalere.reset(); gripers-=2;}
                if (gamepad1.dpad_right){controlor=true;controalere.reset(); gripers+=2;}

                if (gripers > 5) gripers = 5;
                if (gripers < 0) gripers = 0;
            }
            if(controlor==true)
            {
                if(controalere.seconds()>0.5)controlor=false;
            }
            if(schema==0) {
                if (gamepad1.right_bumper) {
                    colectare.griper_stanga.setPosition(colectare.griper_stanga_close);
                }
                if (gamepad1.left_bumper) {
                    colectare.griper_dreapta.setPosition(colectare.griper_dreapta_close);
                }
            }
            if(gamepad2.right_trigger==0)
                if(gamepad2.touchpad){
                    colectare.state_reset();
                    gripers=3;
                    culis_miscare=false;
                    schema=0;}
            if(gamepad2.right_trigger!=0)
                if(gamepad2.touchpad) {
                colectare.state_reset_alt();
                gripers=3;
                    culis_miscare=false;
                schema=0;
            }
            if((gamepad2.triangle
                    || (colectare.dreapta.getDistance(DistanceUnit.CM)<1
                    && colectare.stanga.getDistance(DistanceUnit.CM) < 1))
                    && mediu==false
                    && colectare.extindere.getCurrentPosition()<50 &&
                    colectare.culisanta.getCurrentPosition()<20 && schema==1
            ) {
                colectare.griper_stanga.setPosition(colectare.griper_stanga_close);
                colectare.griper_dreapta.setPosition(colectare.griper_dreapta_close);
                colectare.rotite.setPosition(colectare.rotite_colect);
                colectare.griper_rotatie.setPosition(colectare.rot_drept);
                colectare.colection(teto);
                mediu=true;
            }

            if(mediu==true) {
                colectare.colection2(teto);
//                if(teto.seconds()>2.3 && teto.seconds()<2.4)colectare.score();
                if(teto.seconds()>2.1){mediu=false;perius=true; periuta.reset();}
            }

            if(gamepad2.square) {colectare.score(); schema=0;}

            if(gamepad2.right_bumper && colectare.culisanta_pozitie<2400)colectare.culisanta_pozitie+=20;
            if(gamepad2.left_bumper && colectare.culisanta_pozitie>0)colectare.culisanta_pozitie-=20;

            if(gamepad1.right_trigger!=0){colectare.extindere_state=1;}
            if(gamepad1.left_trigger!=0){colectare.extindere_state=1;}
            if(gamepad1.left_trigger==0 && gamepad1.right_trigger==0) colectare.extindere_state=0;

            if (mediu == false && perius==false)  {


                if (gamepad2.left_stick_y != 0) {
                    colectare.perie.setPower(-1);

                    colect = true;
                }

                if (gamepad2.right_stick_y != 0) {
                    colectare.perie.setPower(1);
                    colect = true;
                }
            }
            if(perius==false){
            if(gamepad2.right_stick_y ==0 && gamepad2.left_stick_y ==0){

                colect=false;
                colectare.perie.setPower(0);
            }}
            if(gamepad2.right_trigger==0) if(colect==true) colectare.perie_pozitie.setPosition(colectare.perie_pozitie_colect);
            else{if(colect==true) colectare.perie_pozitie.setPosition(colectare.stack4);

            }
            if(colect==false) colectare.perie_pozitie.setPosition(colectare.perie_pozitie_suspensie);

            if(colectare.transfer_dreapta.getPosition()<0.3)schema=0;
            else schema=1;

            if(endgame) {
                if(gamepad1.triangle){
                    colectare.drone.setPosition(colectare.drone_launch);
                }
                if (gamepad2.share) {
                    colectare.agatare_poz();
                    agatatu=13000;
                }
                {
                    if (gamepad2.dpad_down) {
                        colectare.agatare();
                        agatatu = 7000;
                    }
                }
                if(gamepad2.dpad_left){ agatatu-=50; colectare.agatare_man(agatatu);}
                if(gamepad2.dpad_right){ agatatu+=50; colectare.agatare_man(agatatu);}
//                if(gamepad1.dpad_up){
//                    endtindere+=20;
//
//                }
//                if(gamepad1.dpad_up){
//                    endtindere-=20;
//
//                }

            }
            if(perius==true && periuta.seconds()>0.4 )
            {
                perius=false;
            }
            if(perius==true)colectare.perie.setPower(1);
            if(schema==0)colectare.graiper(gripers);
            if(gamepad1.touchpad&& gamepad2.touchpad){
                if(gamepad2.left_trigger!=0){
                    colectare.perie_pozitie_colect-=0.005;
                }
                if(gamepad2.right_trigger!=0)                    colectare.perie_pozitie_colect+=0.005;
                endgame=false;
            }
            colectare.extindere();
            if(mediu==true)colectare.perie.setPower(0);
            colectare.culisante();
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
            telemetry.addData("pozitie",drive.getPoseEstimate());
            telemetry.addData("mediu",mediu);
            telemetry.addData("schema",schema);
            telemetry.addData("teto",teto.seconds());
            telemetry.addData("senzor_st",colectare.stanga.getDistance(DistanceUnit.CM));
            telemetry.addData("senzor_dr",colectare.dreapta.getDistance(DistanceUnit.CM));
            telemetry.addData("senzor_cupa",colectare.distanta_cupa.getDistance(DistanceUnit.CM));

            telemetry.update();
        }
    }
}
