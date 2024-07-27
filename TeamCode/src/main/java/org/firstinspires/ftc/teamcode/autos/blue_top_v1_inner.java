//package org.firstinspires.ftc.teamcode.autos;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.pepeop.colectarev2;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Autonomous(name="blue_top_v1_inner")
//public class blue_top_v1_inner extends LinearOpMode {
//    public  oto_blu oto = new oto_blu(hardwareMap);
//    ElapsedTime timez = new ElapsedTime();
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        colectarev2 colectarev2 = new colectarev2(hardwareMap);
//        ElapsedTime exti = new ElapsedTime();
//        boolean ext=false;
//        ElapsedTime cronos = new ElapsedTime();
//        //        detection_blue detection = new detection_blue(hardwareMap);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(oto.start_blue_top);
//        colectarev2.auto_init_top();
//        TrajectorySequence unu_s = drive.trajectorySequenceBuilder(oto.start_blue_top)
//
//                .lineToSplineHeading(oto.preload_blue_top_stanga)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                    colectarev2.perie_pozitie.setPosition(colectarev2.perie_pozitie_suspensie);
//                })
//
//
//                .lineToSplineHeading(oto.table_align_blue_stanga)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, colectarev2::score)
//                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()->{
//                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept_invers);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0, () ->
//                {
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.3, colectarev2::state_reset)
//                .lineToSplineHeading(oto.front_trav_blue)
//                // ciclu 2
//                .lineToSplineHeading(oto.back_trav_blue)
//                .build();
////                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
////                    colectarev2.stack54(exti,ext);
////                })
//
//        TrajectorySequence unu_s2 = drive.trajectorySequenceBuilder(oto.back_trav_blue)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
//                    colectarev2.transfer(colectarev2.dreapta_hopa);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_default);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_inter);
//                })
//                .lineToSplineHeading(oto.front_trav_blue)
//
//                .lineToSplineHeading(oto.blue_stack_score_stanga)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.4, ()-> {
//                    colectarev2.state_reset();
//                })  .UNSTABLE_addTemporalMarkerOffset(-0.7, ()-> {
//                    colectarev2.score();
//                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept);
//                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_orizont_invers);
//                    colectarev2.culisanta_pozitie=400;
//                    colectarev2.culisante();
//
//                })
//                .lineToSplineHeading(oto.front_trav_blue)
////                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
////                    colectarev2.extindere_state=1;
////                    colectarev2.extindere();
////                })
//                .lineToSplineHeading(oto.back_trav_blue)
//
//                .build();
//        TrajectorySequence unu_s3 = drive.trajectorySequenceBuilder(oto.back_trav_blue)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
//                    colectarev2.transfer(colectarev2.dreapta_hopa);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_default);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_inter);
//                })
//                .lineToSplineHeading(oto.front_trav_blue)
//
//                .lineToSplineHeading(oto.blue_stack_score_stanga)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_close);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.4, ()-> {
//                    colectarev2.state_reset();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()-> {
//                    colectarev2.score();
//                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_orizont_invers);
//                    colectarev2.culisanta_pozitie=400;
//                    colectarev2.culisante();
//                })
//                .lineToSplineHeading(oto.table_blue_park)
//
//                .build();
//
//
//
//
//        TrajectorySequence unu_m = drive.trajectorySequenceBuilder(oto.start_blue_top)
//
//                .lineToSplineHeading(oto.preload_blue_top_mid)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                    colectarev2.perie_pozitie.setPosition(colectarev2.perie_pozitie_suspensie);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, colectarev2::score)
////                .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
////                    colectarev2.griper_rotatie.setPosition(colectarev2.rot_drept_invers);
////                })
//
//                .lineToSplineHeading(oto.table_align_blue_mid)
//                .UNSTABLE_addTemporalMarkerOffset(0, () ->
//                {
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                })
//                // ciclu 2
//                .UNSTABLE_addTemporalMarkerOffset(0.3, colectarev2::state_reset)
//                .lineToSplineHeading(oto.back_trav_stanga_mid)
//                .build();
////                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
////                    colectarev2.stack54(exti,ext);
////                })
//
//        TrajectorySequence unu_m1 = drive.trajectorySequenceBuilder(oto.back_trav_stanga_mid)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
//                    colectarev2.transfer(colectarev2.dreapta_hopa);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_default);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_inter);
//                })
//                .lineToSplineHeading(oto.table_align_blue_mid)
//                .UNSTABLE_addTemporalMarkerOffset(0, () ->
//                {
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () ->
//                {
//                    colectarev2.state_reset();
//                })  .UNSTABLE_addTemporalMarkerOffset(-0.7, () ->
//                {
//                    colectarev2.score();
//                })
//                .lineToSplineHeading(oto.back_trav_stanga_mid)
//                .build();
//
////                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
////                    colectarev2.stack54(exti,ext);
////                })
//        TrajectorySequence unu_m2 = drive.trajectorySequenceBuilder(oto.back_trav_stanga_mid)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
//                    colectarev2.transfer(colectarev2.dreapta_hopa);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_default);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_inter);
//                })
//
//                .lineToSplineHeading(oto.table_align_blue_mid)
//                .UNSTABLE_addTemporalMarkerOffset(0, () ->
//                {
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.7, () ->
//                {
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () ->
//                {
//                    colectarev2.state_reset();
//                })
//                .lineToSplineHeading(oto.table_blue_park)
//                .build();
//
//
//
//
//
//        TrajectorySequence unu_d = drive.trajectorySequenceBuilder(oto.start_blue_top)
//
//                .lineToSplineHeading(oto.preload_blue_top_stanga_outer)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                    colectarev2.perie_pozitie.setPosition(colectarev2.perie_pozitie_suspensie);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, colectarev2::score)
//
//                .lineToSplineHeading(oto.table_align_blue_stanga)
//                .UNSTABLE_addTemporalMarkerOffset(0, () ->
//                {
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                })
//                .lineToSplineHeading(oto.front_trav_stanga)
//                // ciclu 2
//                .UNSTABLE_addTemporalMarkerOffset(-0.3, colectarev2::state_reset)
//                .lineToSplineHeading(oto.back_trav_stanga)
//
//                .turn(Math.toRadians(-30))
//                .build();
////                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
////                    colectarev2.stack54(exti,ext);
////                })
//
//        TrajectorySequence unu_d1 = drive.trajectorySequenceBuilder(oto.back_trav_stanga_schema)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
//                    colectarev2.transfer(colectarev2.dreapta_hopa);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_default);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_inter);
//                })
//
//                .lineToSplineHeading(oto.front_trav_stanga)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2.capac.setPosition(colectarev2.capac_colectare);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.7, colectarev2::score)
//                .lineToSplineHeading(oto.blue_stack_score_stanga)
//                .UNSTABLE_addTemporalMarkerOffset(0, () ->
//                {
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () ->
//                {
//                    colectarev2.state_reset();
//                })
//                .lineToSplineHeading(oto.front_trav_stanga)
//
//                .lineToSplineHeading(oto.back_trav_stanga)
//                .turn(Math.toRadians(-30))
//                .build();
//
////                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
////                    colectarev2.stack54(exti,ext);
////                })
//        TrajectorySequence unu_d2 = drive.trajectorySequenceBuilder(oto.back_trav_stanga_schema)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2.capac.setPosition(colectarev2.capac_scorare);
//                    colectarev2.transfer(colectarev2.dreapta_hopa);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_default);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);
//                    colectarev2. griper_stanga.setPosition(    colectarev2. griper_stanga_open);                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> {
//                    colectarev2.transfer(colectarev2.dreapta_inter);
//                })
//
//
//
//                .lineToSplineHeading(oto.front_trav_stanga)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
//                    colectarev2.capac.setPosition(colectarev2.capac_colectare);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.7, colectarev2::score)
//                .lineToSplineHeading(oto.blue_stack_score_stanga)
//                .UNSTABLE_addTemporalMarkerOffset(0, () ->
//                {
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                    colectarev2.griper_stanga.setPosition(colectarev2.griper_stanga_close);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () ->
//                {
//                    colectarev2.state_reset();
//                })
//                .lineToSplineHeading(oto.table_blue_park)
//                .build();
//
//
//        // stanga
//        //        if(!isStarted())
//        //        {
//        //            while(!isStarted()){
//        //                detection.wallhack();
//        //                telemetry.addData("case",detection.d);
//        //                telemetry.update();sleep(20);}
//        //        }
//
//        waitForStart();
//        if(isStopRequested()) return;
//        //3= stanga
//        int caz
////                =detection.d
//                = 3;
//
//        colectarev2.transfer(colectarev2.transfer_preload);
//        colectarev2.rotite.setPosition(colectarev2.rotite_preload);
//        if(caz==1) {
//            drive.followTrajectorySequence(unu_s);
////        ext = true;
////        while (ext == true) {
////            colectarev2.stack54(cronos, ext);
////        }
////        colectarev2.perie.setPower(0);
//
//            drive.followTrajectorySequence(unu_s2);
////        ext = true;
////        while (ext == true) {
////            colectarev2.stack54(cronos, ext);
////        }
////        colectarev2.perie.setPower(0);
//
//            drive.followTrajectorySequence(unu_s3);
//        }
//        if(caz==2){
//            drive.followTrajectorySequence(unu_m);
////        ext = true;
////        while (ext == true) {
////            colectarev2.stack54(cronos, ext);
////        }
////        colectarev2.perie.setPower(0);
//
//            drive.followTrajectorySequence(unu_m1);
////        ext = true;
////        while (ext == true) {
////            colectarev2.stack54(cronos, ext);
////        }
////        colectarev2.perie.setPower(0);
//
//            drive.followTrajectorySequence(unu_m2);
//        }
//        if(caz==3){
//            drive.followTrajectorySequence(unu_d);
//            ext = true;
////        while (ext == true) {
////            colectarev2.stack54(cronos, ext);
////        }
////        colectarev2.perie.setPower(0);
//
//            drive.followTrajectorySequence(unu_d1);
////        ext = true;
////        while (ext == true) {
////            colectarev2.stack54(cronos, ext);
////        }
////        colectarev2.perie.setPower(0);
//
//            drive.followTrajectorySequence(unu_d2);
//        }
//        while(!isStopRequested() && opModeIsActive()){
//            colectarev2.state_reset();
//        }
//    }}
