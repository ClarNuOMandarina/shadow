package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pepeop.colectarev2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="blue_top_v1")
public class blue_top_v1 extends LinearOpMode {
    public  oto_blue oto = new oto_blue(hardwareMap);
    ElapsedTime timez = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        colectarev2 colectarev2 = new colectarev2(hardwareMap);
        ElapsedTime exti = new ElapsedTime();
        boolean ext=false;
        ElapsedTime cronos = new ElapsedTime();
        double transfer =1;
        double transfer_m=1;
        TrajectoryAccelerationConstraint accel = new ProfileAccelerationConstraint(30);

        //        detection_red detection = new detection_red(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(oto.start_red_top);
        colectarev2.auto_init_top();
        TrajectorySequence unu_s = drive.trajectorySequenceBuilder(oto.start_red_top)

                .lineToSplineHeading(oto.preload_red_top_stanga)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    colectarev2.score();
                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept

                    );
                    colectarev2.culisanta_pozitie=150;
                    colectarev2.culisante();
//                    colectarev2.rotite.setPosition(colectarev2.rotite_score_preload);
                })
                .lineToSplineHeading(oto.table_align_red_stanga_top)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.parcare_colt_red)

                .build();
        TrajectorySequence unu_s2 = drive.trajectorySequenceBuilder(oto.back_trav_dreapta_schema)
                .turn(Math.toRadians(-29))
                .UNSTABLE_addTemporalMarkerOffset(0+transfer, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_hopa);
                    colectarev2.perie.setPower(0);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.2+transfer, ()-> {
                    colectarev2.capac.setPosition(colectarev2.capac_scorare);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.6+transfer, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_default);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1+transfer, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_open);
                    colectarev2.perie.setPower(1);

                })
                .UNSTABLE_addTemporalMarkerOffset(1.6+transfer, ()-> {
                    colectarev2.score();
                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_orizont_invers);
                    colectarev2.culisanta_pozitie=1600;
                    colectarev2.culisante();                })
                .UNSTABLE_addTemporalMarkerOffset(3+transfer, ()-> {

                    colectarev2.perie.setPower(0);

                })
                .lineToSplineHeading(oto.front_trav_dreapta)

                .lineToSplineHeading(oto.red_stack_score_dreapta)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {

                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.front_trav_dreapta)

                .lineToSplineHeading(oto.back_trav_dreapta)
                .turn(Math.toRadians(29))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.extindere_state=1;
                    colectarev2.extindere();
                    colectarev2.perie_pozitie.setPosition(colectarev2.stack3);
                    colectarev2.perie.setPower(-1);

                })
                .forward(2)
                .build();

        TrajectorySequence unu_s3 = drive.trajectorySequenceBuilder(oto.back_trav_dreapta_schema)
                .turn(Math.toRadians(-29))
                .UNSTABLE_addTemporalMarkerOffset(0+transfer, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_hopa);
                    colectarev2.perie.setPower(0);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.2+transfer, ()-> {
                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6+transfer, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_default);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1+transfer, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_open);
                    colectarev2.perie.setPower(-1);

                })
                .UNSTABLE_addTemporalMarkerOffset(1.6+transfer, ()-> {
                    colectarev2.score();
                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_orizont_invers);
                    colectarev2.culisanta_pozitie=1600;
                    colectarev2.culisante();                })
                .UNSTABLE_addTemporalMarkerOffset(3+transfer, ()-> {

                    colectarev2.perie.setPower(0);

                })
                .lineToSplineHeading(oto.front_trav_dreapta)

                .lineToSplineHeading(oto.red_stack_score_dreapta)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {

                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.table_align_red_mid)
                .build();

        TrajectorySequence unu_m = drive.trajectorySequenceBuilder(oto.start_red_top)

                .lineToSplineHeading(oto.preload_red_top_mid)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    colectarev2.score();
                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept

                    );
                    colectarev2.culisanta_pozitie=150;
                    colectarev2.culisante();
//                    colectarev2.rotite.setPosition(colectarev2.rotite_score_preload);
                })
                .lineToSplineHeading(oto.table_align_red_mid)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.parcare_colt_red)

                .build();
        TrajectorySequence unu_m2 = drive.trajectorySequenceBuilder(oto.back_trav_dreapta_mid)
                .setAccelConstraint(accel)
                .UNSTABLE_addTemporalMarkerOffset(0+transfer_m, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_hopa);
                    colectarev2.perie.setPower(0);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.2+transfer_m, ()-> {
                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6+transfer_m, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_default);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1+transfer_m, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_open);                })
                .UNSTABLE_addTemporalMarkerOffset(1.35+transfer_m, ()-> {
                    colectarev2.perie.setPower(1);
                    colectarev2.score();
                    colectarev2.culisanta_pozitie=1000;
                    colectarev2.culisante();
                })



                .lineToSplineHeading(oto.table_align_red_mid_v2)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.perie.setPower(0);
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.back_trav_dreapta_mid_schema)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.extindere_state=1;
                    colectarev2.extindere();
                    colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
                    colectarev2.perie.setPower(-1);
                })
                .lineToSplineHeading(oto.back_trav_dreapta_mid)


                .build();
        TrajectorySequence unu_m3 = drive.trajectorySequenceBuilder(oto.back_trav_dreapta_mid)
                .setAccelConstraint(accel)
                .UNSTABLE_addTemporalMarkerOffset(0+transfer_m, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_hopa);
                    colectarev2.perie.setPower(0)
                    ;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2+transfer_m, ()-> {
                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6+transfer_m, ()-> {
                    colectarev2.transfer(colectarev2.dreapta_default);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1+transfer_m, ()-> {
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_open);                })
                .UNSTABLE_addTemporalMarkerOffset(1.35+transfer_m, ()-> {
                    colectarev2.perie.setPower(1);
                    colectarev2.score();
                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept);
                    colectarev2.culisanta_pozitie=1000;
                    colectarev2.culisante();                })


                .lineToSplineHeading(oto.table_align_red_mid_v2)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    colectarev2.perie.setPower(0);
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.table_align_red_stanga)

                .build();



        TrajectorySequence unu_d = drive.trajectorySequenceBuilder(oto.start_red_top)

                .lineToSplineHeading(oto.preload_red_top_dreapta)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    colectarev2.score();
                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept_invers

                    );
                    colectarev2.culisanta_pozitie=150;
                    colectarev2.culisante();
//                    colectarev2.rotite.setPosition(colectarev2.rotite_score_preload);
                })
                .lineToSplineHeading(oto.table_align_red_dreapta_top)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
                    colectarev2. griper_dreapta.setPosition(    colectarev2. griper_dreapta_close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    colectarev2.state_reset();
                })
                .lineToSplineHeading(oto.parcare_colt_red)

                .build();


        CameraDetector_blue_top camera = new CameraDetector_blue_top(OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")));
        CameraDetector_blue_top.Result result = CameraDetector_blue_top.Result.NONE;
        while (opModeInInit()) {
            result = camera.detect();
            telemetry.addLine("Location" + result);
            telemetry.addData("cazus",camera.cazus);

            telemetry.update();
        }
        int caz=1;
        caz=camera.cazus;
        camera.stop();
        waitForStart();
        if(isStopRequested()) return;
        //3= stanga


        colectarev2.transfer(colectarev2.transfer_preload);
        colectarev2.rotite.setPosition(colectarev2.rotite_preload);
        if(caz==1) {
            drive.followTrajectorySequence(unu_s);

//        ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);

        }
        if(caz==2){
            drive.followTrajectorySequence(unu_m);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack5);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<1.5){}
            colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<2)
            {   }
            colectarev2.extindere_state=0;
            colectarev2.extindere();
//        ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);

            drive.followTrajectorySequence(unu_m2);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack3);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<1.5)
                colectarev2.perie_pozitie.setPosition(colectarev2.stack2);
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<2)
            {   }
            colectarev2.extindere_state=0;
            colectarev2.extindere();
//            ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);
            drive.followTrajectorySequence(unu_m3);

        }
        if(caz==3){
            drive.followTrajectorySequence(unu_d);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack5);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<1.5)
                colectarev2.perie_pozitie.setPosition(colectarev2.stack4);
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<2)
            {   }
            colectarev2.extindere_state=0;
            colectarev2.extindere();
//        ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);

            drive.followTrajectorySequence(unu_s2);
            colectarev2.perie_pozitie.setPosition(colectarev2.stack3);
            colectarev2.perie.setPower(-1);
            cronos.reset();
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<1.5)
                colectarev2.perie_pozitie.setPosition(colectarev2.stack2);
            while((colectarev2.dreapta.getDistance(DistanceUnit.CM)>1 || colectarev2.stanga.getDistance(DistanceUnit.CM)>1)&& cronos.seconds()<2)
            {   }
            colectarev2.extindere_state=0;
            colectarev2.extindere();
//            ext = true;
//        while (ext == true) {
//            colectarev2.stack54(cronos, ext);
//        }
//        colectarev2.perie.setPower(0);
            drive.followTrajectorySequence(unu_s3);

        }
        while(!isStopRequested() && opModeIsActive()){
            colectarev2.state_reset();
        }
    }}
