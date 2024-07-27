package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class oto_blue {
    public double x_offest=0;
    public double y_offest=0;

    public Pose2d start_red_bot = new Pose2d(-40+x_offest,62+y_offest,Math.toRadians(-270));
    public Pose2d start_red_top = new Pose2d( 16.1+x_offest,61.3+y_offest,Math.toRadians(-270));
    public Pose2d start_red_mid = new Pose2d( -35.3+x_offest,61+y_offest,Math.toRadians(-270));


    Pose2d preload_red_bot_dreapta = new Pose2d(-56  +x_offest,24+y_offest,Math.toRadians(-142));
    Pose2d preload_red_bot_mid = new Pose2d(-51+x_offest,21+y_offest,Math.toRadians(-154.5));
    Pose2d preload_red_bot_stanga = new Pose2d(-35.8+x_offest,28+y_offest,Math.toRadians(-155));

    Pose2d preload_red_top_stanga = new Pose2d(24+x_offest,46+y_offest,Math.toRadians(90));
    Pose2d preload_red_top_mid = new Pose2d(11+x_offest,38+y_offest,Math.toRadians(90));
    Pose2d preload_red_top_dreapta = new Pose2d(-7  +x_offest,46 +y_offest,Math.toRadians(74));
    Pose2d preload_red_mid_stanga = new Pose2d(-34+x_offest,36+y_offest,Math.toRadians(143));
//    Pose2d preload_red_mid_mid = new Pose2d(-35+x_offest,36+y_offest,Math.toRadians(-270));
    Pose2d preload_red_mid_dreapta = new Pose2d(-35+x_offest,36+y_offest,Math.toRadians(48));
    Pose2d preload_red_top_dreapta_outer = new Pose2d(23+x_offest,48+y_offest,Math.toRadians(-270));

    Pose2d preload_red_bot_intermediar= new Pose2d(20+x_offest,32+y_offest,Math.toRadians(-270));
    Pose2d preload_red_bot_intermediar_dreapta= new Pose2d(30+x_offest,34  +y_offest,Math.toRadians(0));
    Pose2d from_preload_to_front_traversal = new Pose2d(-38+x_offest,10+y_offest,Math.toRadians(-90));
    Pose2d mid_top_intermediar = new Pose2d(11+x_offest,39  +y_offest,Math.toRadians(-270));

    Pose2d back_trav_red = new Pose2d(-23+x_offest,12+y_offest,Math.toRadians(-180));
    Pose2d back_trav_dreapta = new Pose2d(17+x_offest,57+y_offest,Math.toRadians(180));
    Pose2d back_trav_dreapta_schema = new Pose2d(-27+x_offest,59+y_offest,Math.toRadians(-150));
    Pose2d back_trav_schema = new Pose2d(-53+x_offest,39+y_offest,Math.toRadians(-180));
    Pose2d back_trav_red_intermediar= new Pose2d(-35+x_offest,39+y_offest,Math.toRadians(90));
    Pose2d back_trav_red_intermediar2= new Pose2d(-35+x_offest,37+y_offest,Math.toRadians(90));


    Pose2d back_trav_perete_intermediu= new Pose2d(-35+x_offest,57+y_offest,Math.toRadians(90));
    Pose2d back_trav_perete_intermediu2= new Pose2d(-32+x_offest,60+y_offest,Math.toRadians(180));


    Pose2d front_trav_red = new Pose2d(12+x_offest,12+y_offest,Math.toRadians(-180));
    Pose2d front_trav_red_schema = new Pose2d(34+x_offest,14+y_offest,Math.toRadians(-180));
    Pose2d front_trav_dreapta = new Pose2d(10+x_offest,58.4+y_offest,Math.toRadians(-180));

    Pose2d front_trav_schema = new Pose2d(23+x_offest,37+y_offest,Math.toRadians(-180));
    Pose2d back_trav_dreapta_mid=new Pose2d(-22+x_offest,37+y_offest,Math.toRadians(-180));
    Pose2d back_trav_dreapta_mid_schema=new Pose2d(10+x_offest,37+y_offest,Math.toRadians(-180));


    Pose2d front_of_table_red = new Pose2d(38+x_offest,37  +y_offest,Math.toRadians(-180));


    Pose2d table_align_red_dreapta =new Pose2d(44+x_offest,31.7+y_offest,Math.toRadians(180));
    Pose2d table_align_red_dreapta_top = new Pose2d(50+x_offest,33+y_offest,Math.toRadians(-180));
    Pose2d table_align_red_mid = new Pose2d(44+x_offest,36.5+y_offest,Math.toRadians(180));
    Pose2d table_align_red_mid_v2 = new Pose2d(50+x_offest,36.5+y_offest,Math.toRadians(-180));
    Pose2d table_align_red_stanga = new Pose2d(44  +x_offest,40+y_offest,Math.toRadians(180));
    Pose2d red_stack_score_stanga = new Pose2d(49  +x_offest,46.5+y_offest,Math.toRadians(-198));
    Pose2d table_align_red_stanga_top =new Pose2d(50  +x_offest,43+y_offest,Math.toRadians(-180));

    Pose2d table2_align_red_mid = new Pose2d(45.5+x_offest,7.5+y_offest,Math.toRadians(180));


    Pose2d red_stack_score_dreapta = new Pose2d(45  +x_offest,30+y_offest,Math.toRadians(-160));
    Pose2d red_stack_score_dreapta_v2 = new Pose2d(44.5  +x_offest,25.3+y_offest,Math.toRadians(-160));
    Pose2d red_stack_score_dreapta_park = new Pose2d(47  +x_offest,24.3+y_offest,Math.toRadians(-157.4));

    Pose2d red_inner_stack_mid= new Pose2d(-57.4  +x_offest,37.2+y_offest,Math.toRadians(-180));
    Pose2d red_inner_stack= new Pose2d(-101  +x_offest,11.5+y_offest,Math.toRadians(180));
    Pose2d red_stack_score_stanga_park = new Pose2d(46  +x_offest,25+y_offest,Math.toRadians(157.4));


    Pose2d table_red_park = new Pose2d(46+x_offest,63+y_offest,Math.toRadians(-180));
    Pose2d table_red_park_2 = new Pose2d(44+x_offest,60+y_offest,Math.toRadians(-180));
    Pose2d table_red_park_3 = new Pose2d(61+x_offest,62+y_offest,Math.toRadians(0));

    Pose2d stack_red = new Pose2d(-58+x_offest,15+y_offest,Math.toRadians(-180));
    Pose2d stack_red_dreapta = new Pose2d(-61.5+x_offest,40.5+y_offest,Math.toRadians(-180));
    Pose2d stack_red_dreapta2 = new Pose2d(-59+x_offest,41+y_offest,Math.toRadians(-180));



    Pose2d parcare_mid_red = new Pose2d(51+x_offest,21+y_offest,Math.toRadians(-180));
    Pose2d parcare_colt_red = new Pose2d(35+x_offest,57+y_offest,Math.toRadians(-180));
    Pose2d parcare_colt_red2 = new Pose2d(45+x_offest,60+y_offest,Math.toRadians(180));

    public oto_blue(HardwareMap hardwareMap)
    {

    }
}
