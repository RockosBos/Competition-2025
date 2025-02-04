package frc.robot;

public class Constants {

    //CAN ID's

    public static final int ID_INTAKE_ROLLER = 16;
    public static final int ID_INTAKE_ROTATE = 17;
    public static final int ID_INTAKE_ELEVATOR = 18;
    public static final int ID_SCORE_ELEVATOR = 19;
    public static final int ID_SCORE_AGITATE = 13;
    public static final int ID_SCORE_CLAW = 12;
    public static final int ID_SCORE_PIVOT = 11;
    public static final int ID_SCORE_ROTATE = 10;

    //Intake Constant Values

    public static final double INTAKE_ROLLER_INFEED_VOLTAGE = 0.5;
    public static final double INTAKE_ROLLER_OUTFEED_VOLTAGE = -0.5;

    public static final double INTAKE_ELEVATOR_FLOOR_INTAKE_POS = 0;
    public static final double INTAKE_ELEVATOR_LOADING_INTAKE_POS = 0;
    public static final double INTAKE_ELEVATOR_HANDOFF_POS = 0;
    public static final double INTAKE_ELEVATOR_L1_POS = 0;

    public static final double INTAKE_ROTATE_FLOOR_INTAKE_POS = 0;
    public static final double INTAKE_ROTATE_LOADING_INTAKE_POS = 0;
    public static final double INTAKE_ROTATE_HANDOFF_POS = 0;
    public static final double INTAKE_ROTATE_L1_POS = 0;

    //Score Arm Constant Values

    public static final double SCORE_ELEVATOR_INTAKE_POSITION = 0;
    public static final double SCORE_ELEVATOR_L2_POS = 0;
    public static final double SCORE_ELEVATOR_L3_POS = 0;
    public static final double SCORE_ELEVATOR_L4_POS = 0;
    public static final double SCORE_HANDOFF_POS = 0;

    public static final double SCORE_ROTATE_LEFT_POS = 0;
    public static final double SCORE_ROTATE_RIGHT_POS = 0;
    public static final double SCORE_ROTATE_CENTER_POS = 0;

    public static final double SCORE_PIVOT_IN_POS = 0;
    public static final double SCORE_PIVOT_OUT_POS = 0;

    public static final double SCORE_CLAW_CLOSED_POS = 0;
    public static final double SCORE_CLAW_OPEN_POS = 0;

    public static final double SCORE_AGITATOR_INFEED_VOLTAGE = 0;
    public static final double SCORE_AGITATOR_OUTFEED_VOLTAGE = 0;

    //Absolute Encoder offsets
    
    public static final double OFFSET_INTAKE_ROTATE_ABS = 0.0;
    public static final double OFFSET_SCORE_ROTATE_ABS = 0.0;
    public static final double OFFSET_SCORE_PIVOT_ABS = 0.0;
    public static final double OFFSET_SCORE_CLAW_ABS = 0.0;

    //thresholds
    public static final double THRESHOLD_INTAKE_ROTATE_POS = 0.01;
    public static final double THRESHOLD_ELEVATOR_INTAKE_POS = 0.01;
    public static final double THRESHOLD_ELEVATOR_SCORE_POS = 0.01;
    public static final double THRESHOLD_SCORE_ROTATE_POS = 0.01;
    public static final double THRESHOLD_SCORE_PIVOT_POS = 0.01;
    public static final double THRESHOLD_SCORE_CLAW_POS = 0.01;


}
