package org.firstinspires.ftc.teamcode.pepeop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class colectarev2 {

    public Servo perie_pozitie, capac;
    public  DcMotorEx perie,extindere,culisanta;
    public double capac_scorare= 0.1360000000000001;
    public double capac_colectare= 0.030500000000000002;
    public double perie_pozitie_stacc= 0.906;

    public double perie_pozitie_colect= 0.198  ;
    public double perie_pozitie_suspensie= 0.16600000000005477;
    public double perie_pozitie_init= 0.1456;
    public double stack5= 0.185;
    public double stack4= 0.1875;
    public double stack3= 0.189;
    public double stack2= 0.192;
    public static  int culisanta_pozitie=0;
    public int extindere_state=0;
    public int extins=1175;
    public int extins_semi=1000;
    public int extins_agatare=250;
    public int max_extins=1175;
    public int retras=0;
    public DistanceSensor stanga,dreapta;

    public Servo transfer_stanga,transfer_dreapta;
    public double dreapta_default=0.6749999999999989;
    public double dreapta_score=0.165;
    public double dreapta_default_inter=0.6169999999999874;
    public double dreapta_inter=0.59;
    public double dreapta_hopa=0.529;
    public double transfer_preload=0;

    public Servo griper_stanga,griper_dreapta,griper_rotatie;
    public double griper_stanga_close=1;
    public double griper_stanga_open=0.5;
    public double griper_dreapta_close=0;
    public double griper_dreapta_open=0.4179999999999999;
    public double rot_drept=0.6514999999999987;
    public double rot_drept_invers=0;
    public double rot_orizont=0.96;
    public double rot_orizont_invers=0.27;

    public double rot_diag_stanga=0.805;
    public double rot_diag_dreapta2=0.135;
    public double rot_diag_dreapta=0.5;
    public DcMotorEx agatare;

    public double agatare_default;
    public double agatare_action;

    public Servo rotite;
    public double rotite_colect= 0.626;
    public double rotite_score=0.2505;
    public double rotite_score_preload=0.547;
    public double rotite_preload=0.04;
    public int neagatat=0;
    public int agatare_trage=7000;
    public int agatat=17500;
    public Servo drone;

    public double drone_launch=0;
    public double drone_init=1;

    ModernRoboticsI2cRangeSensor distanta_cupa;
    public boolean buffer;
    boolean cupa_dist;
    public boolean intermediu,autos;
    public boolean coco;
    public static final   double offset=-0.4;
    public colectarev2(HardwareMap hardwareMap) {

        perie = hardwareMap.get(DcMotorEx.class, "perie");
        extindere = hardwareMap.get(DcMotorEx.class, "extindere");
        agatare=hardwareMap.get(DcMotorEx.class,"agatare");
        culisanta = hardwareMap.get(DcMotorEx.class, "culi");

        drone = hardwareMap.get(Servo.class,"drone");

        rotite = hardwareMap.get(Servo.class, "rotite");
        capac = hardwareMap.get(Servo.class, "capac");
        perie_pozitie = hardwareMap.get(Servo.class, "perie_pozitie");
        transfer_dreapta = hardwareMap.get(Servo.class, "t_d");
        transfer_stanga = hardwareMap.get(Servo.class, "t_s");


        griper_dreapta = hardwareMap.get(Servo.class, "griper_dreapta");
        griper_stanga = hardwareMap.get(Servo.class, "griper_stanga");

        griper_rotatie = hardwareMap.get(Servo.class, "griper_rotatie");

        stanga=hardwareMap.get(DistanceSensor.class,"stanga");
        dreapta=hardwareMap.get(DistanceSensor.class,"dreapta");

        distanta_cupa=hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"cupa_s");

        perie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extindere.setDirection(DcMotorSimple.Direction.REVERSE);
        extindere.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extindere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        culisanta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        culisanta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        agatare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        agatare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intermediu = false;
        autos = false;

        buffer = false;
        coco = false;
        cupa_dist=false;
    }

    public void culisante()  {
        culisanta.setTargetPosition(culisanta_pozitie);
        culisanta.setPower(1);
        culisanta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //todo range 13-20
    public void extindere() {
        if (extindere_state == 0){
            extindere.setTargetPosition(retras);
            extindere.setPower(1);
            extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extins=max_extins;
        }
        else{
            extindere.setPower(1);
            extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(cupa_dist){
                if(distanta_cupa.getDistance(DistanceUnit.CM)<12&&extins>=0)extins=extindere.getCurrentPosition()-
                        ((30-(int)distanta_cupa.getDistance(DistanceUnit.CM))*10);
                else if(distanta_cupa.getDistance(DistanceUnit.CM)<11)extins=0;
                else if( Double.isNaN(distanta_cupa.getDistance(DistanceUnit.CM))
                        &&  extins>0)extins=extindere.getCurrentPosition()-500;
                else if (distanta_cupa.getDistance(DistanceUnit.CM)>25 && distanta_cupa.getDistance(DistanceUnit.CM)<40)
                    extins=extindere.getCurrentPosition()+((int)distanta_cupa.getDistance(DistanceUnit.CM)*5);
                else if (distanta_cupa.getDistance(DistanceUnit.CM)>30)extins=max_extins;
            }
            extindere.setTargetPosition(extins);

        }
    }
    public void transfer(double opa){

        transfer_dreapta.setPosition(opa);
        transfer_stanga.setPosition(opa);
    }
    public void colection(ElapsedTime crono)
    {

        crono.reset();
        if(crono.seconds()<0.2){
            transfer_dreapta.setPosition(dreapta_hopa);
            transfer_stanga.setPosition(dreapta_hopa);}}
    public void colection2( ElapsedTime crono){

        if(crono.seconds()<0.6+offset){
            capac.setPosition(capac_scorare);
        }
        if(crono.seconds()>(1.2+offset) && crono.seconds()<(1.6+offset))
        {
            transfer_dreapta.setPosition(dreapta_default);
            transfer_stanga.setPosition(dreapta_default);

        }
//        if(crono.seconds()>(1.2+offset) && crono.seconds()<(1.4+offset))
//        {
//
//            transfer_dreapta.setPosition(dreapta_default);
//            transfer_stanga.setPosition(dreapta_default);
//        }
        if(crono.seconds()>=(1.6+offset) && crono.seconds()<(2.1+offset)){
            griper_stanga.setPosition(griper_stanga_open);
            griper_dreapta.setPosition(griper_dreapta_open);
        }
        if(crono.seconds()>(2.1+offset) && crono.seconds()<(2.4+offset)){
            transfer_dreapta.setPosition(dreapta_inter);
            transfer_stanga.setPosition(dreapta_inter);
            intermediu=false;

        }

    }

    public void tele_init()
    {
        drone.setPosition(drone_init);
        agatare.setTargetPosition(neagatat);
        agatare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agatare.setPower(1);
        rotite.setPosition(rotite_colect);
        transfer_stanga.setPosition(dreapta_inter);
        transfer_dreapta.setPosition(dreapta_inter);

        griper_stanga.setPosition(griper_stanga_close);
        griper_dreapta.setPosition(griper_dreapta_close);

        griper_rotatie.setPosition(rot_drept);
        capac.setPosition(capac_colectare);
        perie_pozitie.setPosition(perie_pozitie_init);
    }
    public void auto_init_bot()
    {
        rotite.setPosition(rotite_colect);
        transfer_stanga.setPosition(dreapta_hopa);
        transfer_dreapta.setPosition(dreapta_hopa);

        griper_stanga.setPosition(griper_stanga_open);
        griper_dreapta.setPosition(griper_dreapta_close);

        griper_rotatie.setPosition(rot_drept);
        capac.setPosition(capac_colectare);
        perie_pozitie.setPosition(perie_pozitie_init);
    }
    public void auto_init_top()
    {
        drone.setPosition(drone_init);
        rotite.setPosition(rotite_colect);
        transfer_stanga.setPosition(dreapta_hopa);
        transfer_dreapta.setPosition(dreapta_hopa);

        griper_stanga.setPosition(griper_stanga_open);
        griper_dreapta.setPosition(griper_dreapta_open);

        griper_rotatie.setPosition(rot_drept);
        capac.setPosition(capac_colectare);
        perie_pozitie.setPosition(perie_pozitie_init);

        extindere.setPower(1);
        extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extindere.setTargetPosition(retras);
    }

    public void state_reset(){
        griper_stanga.setPosition(griper_stanga_close);
        griper_dreapta.setPosition(griper_dreapta_close);
        rotite.setPosition(rotite_colect);
        griper_rotatie.setPosition(rot_drept);
        culisanta_pozitie = 0;
        transfer_stanga.setPosition(dreapta_inter);
        transfer_dreapta.setPosition(dreapta_inter);
        capac.setPosition(capac_colectare);
        culisante();
    }
    public void state_reset_alt(){
        rotite.setPosition(rotite_colect);
        griper_rotatie.setPosition(rot_drept);
        culisanta_pozitie = 0;
        transfer_stanga.setPosition(dreapta_inter);
        transfer_dreapta.setPosition(dreapta_inter);
    }

    public void score() {

        transfer_dreapta.setPosition(dreapta_score);
        transfer_stanga.setPosition(dreapta_score);
        rotite.setPosition(rotite_score);

    }
    public void preload(){
        transfer(transfer_preload);
        rotite.setPosition(rotite_preload);
    }

//    public void stack54(ElapsedTime timer,boolean aha){
//        perie_pozitie.setPosition(stack5);
//        extindere.setTargetPosition(extins_semi);
//        extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        extindere.setPower(1);
//        perie.setPower(1);
//        timer.reset();
//        if(aha)
//        {
//            if(timer.seconds()>1){
//                perie_pozitie.setPosition(stack4);
//            }
//            if(timer.seconds()>1.5){
//                perie.setPower(-1);
//                extindere.setTargetPosition(retras);
//                extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                extindere.setPower(1);
//                aha=false;
//            }
//        }
//    }
    public void agatare_poz(){
        agatare.setTargetPosition(agatat);
        agatare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agatare.setPower(1);

//        extindere.setTargetPosition(extins_agatare);
//        extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        extindere.setPower(1);
    }
    public void agatare(){
        agatare.setTargetPosition(agatare_trage);
        agatare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agatare.setPower(1);    }
    public void agatare_man( int agatare_trag)
    {
        agatare.setTargetPosition(agatare_trag);
        agatare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agatare.setPower(1);    }

    public void graiper(int varus){
        if(varus==3)griper_rotatie.setPosition(rot_drept);
        if(varus==4)griper_rotatie.setPosition(rot_diag_stanga);
        if(varus==5)griper_rotatie.setPosition(rot_orizont);
        if(varus==2)griper_rotatie.setPosition(rot_diag_dreapta);
        if(varus==1)griper_rotatie.setPosition(rot_orizont_invers);
    }

//    public void lansare(){
//        drone.setPosition();
//    }
}
