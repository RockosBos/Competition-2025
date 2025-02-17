package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Constants {

    //CAN ID's

    public static final int ID_INTAKE_ROLLER = 16;
    public static final int ID_INTAKE_ROTATE = 17;
    public static final int ID_INTAKE_ELEVATOR = 18;
    public static final int ID_SCORE_ELEVATOR = 19;
    public static final int ID_SCORE_AGITATE = 13;
    public static final int ID_SCORE_CLAW = 12;
    public static final int ID_SCORE_ROTATE = 11;
    public static final int ID_SCORE_PIVOT = 10;
    public static final int ID_CLIMB_RIGHT = 9;
    public static final int ID_CLIMB_LEFT = 8;
    
    
    public static final int LASERCAN_INTAKE_LEFT = 51;
    public static final int LASERCAN_INTAKE_RIGHT = 52;

    //Intake Constant Values

    public static final double INTAKE_ROLLER_INFEED_VOLTAGE = 0.5;
    public static final double INTAKE_ROLLER_OUTFEED_VOLTAGE = -0.5;
    

    public static final double INTAKE_ELEVATOR_FLOOR_INTAKE_POS = 10.0;
    public static final double INTAKE_ELEVATOR_LOADING_INTAKE_POS = 23.0;
    public static final double INTAKE_ELEVATOR_HANDOFF_POS = 25.0;
    public static final double INTAKE_ELEVATOR_L1_POS = 35;

    public static final double INTAKE_ROTATE_FLOOR_INTAKE_POS = 0.599;
    public static final double INTAKE_ROTATE_LOADING_INTAKE_POS = 0.332;
    public static final double INTAKE_ROTATE_HANDOFF_POS = 0.248;
    public static final double INTAKE_ROTATE_L1_POS = 0;

    //Score Arm Constant Values

    //Lowest Position at 0
    //Highest Position at 160
    public static final double SCORE_ELEVATOR_INTAKE_POSITION = 5;
    public static final double SCORE_ELEVATOR_L2_POS = 155;
    public static final double SCORE_ELEVATOR_L3_POS = 0;
    public static final double SCORE_ELEVATOR_L4_POS = 0;
    public static final double SCORE_HANDOFF_POS = 0;

    public static final double SCORE_ROTATE_LEFT_POS = 0;
    public static final double SCORE_ROTATE_RIGHT_POS = 0;
    public static final double SCORE_ROTATE_CENTER_POS = 0;

    public static final double SCORE_PIVOT_IN_POS = 0;
    public static final double SCORE_PIVOT_OUT_LEFT_POS = 0;
    public static final double SCORE_PIVOT_OUT_RIGHT_POS = 0;

    public static final double SCORE_CLAW_CLOSED_POS = 0;
    public static final double SCORE_CLAW_OPEN_POS = 0;

    public static final double SCORE_AGITATOR_INFEED_VOLTAGE = 0;
    public static final double SCORE_AGITATOR_OUTFEED_VOLTAGE = 0;

    //Absolute Encoder offsets

    public static final double OFFSET_INTAKE_ROTATE_ABS = 0.430 - 0.25;
    public static final double OFFSET_SCORE_ROTATE_ABS = 0.228 - 0.20;
    public static final double OFFSET_SCORE_PIVOT_ABS = 0.522 - 0.5;
    public static final double OFFSET_SCORE_CLAW_ABS = 0.505 - 0.25;

    //Motor Configuration Constants

    public static final IdleMode IDLEMODE_INTAKE_ELEVATOR = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_SCORE_ELEVATOR = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_INTAKE_ROTATE = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_INTAKE_ROLLER = IdleMode.kCoast;
    public static final IdleMode IDLEMODE_SCORE_ROTATE = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_SCORE_PIVOT = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_SCORE_CLAW = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_SCORE_AGITATOR = IdleMode.kCoast;

    public static final boolean INVERT_INTAKE_ELEVATOR = false;
    public static final boolean INVERT_SCORE_ELEVATOR = false;
    public static final boolean INVERT_INTAKE_ROTATE = false;
    public static final boolean INVERT_INTAKE_ROLLER = false;
    public static final boolean INVERT_SCORE_ROTATE = false;
    public static final boolean INVERT_SCORE_PIVOT = false;
    public static final boolean INVERT_SCORE_CLAW = false;
    public static final boolean INVERT_SCORE_AGITATOR = false;

    public static final boolean INVERT_SCORE_PIVOT_ABS = true;
    public static final boolean INVERT_SCORE_ROTATE_ABS = false;
    public static final boolean INVERT_SCORE_CLAW_ABS = false;
    public static final boolean INVERT_INTAKE_ROTATE_ABS = false;

    public static final double RAMPRATE_INTAKE_ELEVATOR = 0.02;
    public static final double RAMPRATE_SCORE_ELEVATOR = 0.1;
    public static final double RAMPRATE_INTAKE_ROTATE = 0.1;
    public static final double RAMPRATE_INTAKE_ROLLER = 0.1;
    public static final double RAMPRATE_SCORE_ROTATE = 0.1;
    public static final double RAMPRATE_SCORE_PIVOT = 0.1;
    public static final double RAMPRATE_SCORE_CLAW = 0.1;
    public static final double RAMPRATE_SCORE_AGITATOR = 0.1;

    public static final int CURRENTLIMIT_INTAKE_ELEVATOR = 20;
    public static final int CURRENTLIMIT_SCORE_ELEVATOR = 20;
    public static final int CURRENTLIMIT_INTAKE_ROTATE = 20;
    public static final int CURRENTLIMIT_INTAKE_ROLLER = 20;
    public static final int CURRENTLIMIT_SCORE_ROTATE = 20;
    public static final int CURRENTLIMIT_SCORE_PIVOT = 20;
    public static final int CURRENTLIMIT_SCORE_CLAW = 20;
    public static final int CURRENTLIMIT_SCORE_AGITATOR = 20;

    //Closed Loop controller Config

    public static final double P_INTAKE_ELEVATOR = 1.0;
    public static final double MAX_OUTPUT_INTAKE_ELEVATOR = 0.60;
    public static final double MIN_OUTPUT_INTAKE_ELEVATOR = -0.60;

    public static final double P_SCORE_ELEVATOR = 0.05;
    public static final double MAX_OUTPUT_SCORE_ELEVATOR = 0.75;
    public static final double MIN_OUTPUT_SCORE_ELEVATOR = -0.75;

    public static final double P_INTAKE_ROTATE = 0.0;
    public static final double MAX_OUTPUT_INTAKE_ROTATE = 0.0;
    public static final double MIN_OUTPUT_INTAKE_ROTATE = -0.0;

    public static final double P_SCORE_ROTATE = 0.0;
    public static final double MAX_OUTPUT_SCORE_ROTATE = 0.0;
    public static final double MIN_OUTPUT_SCORE_ROTATE = -0.0;

    public static final double P_SCORE_PIVOT = 0.0;
    public static final double MAX_OUTPUT_SCORE_PIVOT = 0.0;
    public static final double MIN_OUTPUT_SCORE_PIVOT = -0.0;

    public static final double P_SCORE_CLAW = 0.0;
    public static final double MAX_OUTPUT_SCORE_CLAW = 0.0;
    public static final double MIN_OUTPUT_SCORE_CLAW = -0.0;

    //thresholds
    public static final double THRESHOLD_INTAKE_ROTATE_POS = 0.01;
    public static final double THRESHOLD_ELEVATOR_INTAKE_POS = 0.01;
    public static final double THRESHOLD_ELEVATOR_SCORE_POS = 0.01;
    public static final double THRESHOLD_SCORE_ROTATE_POS = 0.01;
    public static final double THRESHOLD_SCORE_PIVOT_POS = 0.01;
    public static final double THRESHOLD_SCORE_CLAW_POS = 0.01;


}
