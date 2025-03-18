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
    public static final int ID_CLIMB_ROTATE = 8;
    
    
    public static final int LASERCAN_INTAKE_LEFT = 52;
    public static final int LASERCAN_INTAKE_RIGHT = 51;

    public static final int SCORE_ELEVATOR_PROXIMITY_SENSOR = 0;
    public static final int INTAKE_ELEVATOR_PROXIMITY_SENSOR = 1;

    //Servo Stuff

    public static final int SERVO_ID = 0;

    public static final double SERVO_UNLOCKED = 145.0;
    public static final double SERVO_LOCKED = 75.0;

    //Drive constant Values

    public static final double TIP_PROTECTION_THRESHOLD_ROLL = 10.0;
    public static final double TIP_PROTECTION_THRESHOLD_PITCH = 10.0;

    //Intake Constant Values

    public static final double INTAKE_ROLLER_INFEED_VOLTAGE = 13.0;
    public static final double INTAKE_ROLLER_OUTFEED_VOLTAGE = -13.0;
    public static final double INTAKE_ROLLER_HANDOFF_VOLTAGE = -0.8;
    

    public static final double INTAKE_ELEVATOR_FLOOR_INTAKE_POS = 3.765;
    public static final double INTAKE_ELEVATOR_LOADING_INTAKE_POS = 40.5;
    public static final double INTAKE_ELEVATOR_HANDOFF_POS = 29.0;
    public static final double INTAKE_ELEVATOR_L1_POS = 33.75;
    public static final double INTAKE_ELEVATOR_L1_FAILOP_POS = 26.25;

    public static final double INTAKE_ROTATE_FLOOR_INTAKE_POS = 0.66 - 0.06;
    public static final double INTAKE_ROTATE_LOADING_INTAKE_POS = 0.36 - 0.06;
    public static final double INTAKE_ROTATE_HANDOFF_POS = 0.335 - 0.06;
    public static final double INTAKE_ROTATE_L1_POS = 0.55 - 0.06;
    public static final double INTAKE_ROTATE_L1_FAILOP_POS = 0.42 - 0.06;

    //Score Arm Constant Values

    public static final double SCORE_ELEVATOR_INTAKE_POSITION = 4.8 * 0.6;
    public static final double SCORE_ELEVATOR_GO_AWAY_POSITION = 33.75 * 0.6;
    public static final double SCORE_ELEVATOR_L2_POS = 6.375 * 0.6;
    public static final double SCORE_ELEVATOR_L3_POS = 52.5 * 0.6;
    public static final double SCORE_ELEVATOR_L4_POS = 118.875 * 0.6;
    public static final double SCORE_HANDOFF_POS = 3.25 * 0.6;

    public static final double SCORE_ROTATE_LEFT_POS = 0.197;
    public static final double SCORE_ROTATE_RIGHT_POS = 0.704;
    public static final double SCORE_ROTATE_CENTER_POS = 0.441;

    public static final double SCORE_PIVOT_IN_POS = 0.5;
    public static final double SCORE_PIVOT_OUT_LEFT_POS = 0.38;
    public static final double SCORE_PIVOT_OUT_LEFT_SHALLOW_POS = 0.35;
    public static final double SCORE_PIVOT_OUT_RIGHT_POS = 0.62;
    public static final double SCORE_PIVOT_OUT_RIGHT_SHALLOW_POS = 0.65;

    public static final double SCORE_CLAW_CLOSED_POS = 0.44;
    public static final double SCORE_CLAW_OPEN_POS = 0.12;
    public static final double SCORE_CLAW_RELEASE_POS = 0.375;

    public static final double SCORE_AGITATOR_INFEED_VOLTAGE = 12.0;
    public static final double SCORE_AGITATOR_OUTFEED_VOLTAGE = -12.0;

    //Climb Constant Values

    public static final double CLIMB_IN_POS = 0.0;
    public static final double CLIMB_CLIMBING_POS = 160.0 * 0.555;
    public static final double CLIMB_CAPTURE_POS = 383.0 * 0.555;

    //Absolute Encoder offsets

    public static final double OFFSET_INTAKE_ROTATE_ABS = 0.3297 - 0.25;
    public static final double OFFSET_SCORE_ROTATE_ABS = 0.7756 - 0.20;
    public static final double OFFSET_SCORE_PIVOT_ABS = 0.23 + 0.5;
    public static final double OFFSET_SCORE_CLAW_ABS = 0.595 - 0.1;

    //Motor Configuration Constants

    public static final IdleMode IDLEMODE_INTAKE_ELEVATOR = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_SCORE_ELEVATOR = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_INTAKE_ROTATE = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_INTAKE_ROLLER = IdleMode.kCoast;
    public static final IdleMode IDLEMODE_SCORE_ROTATE = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_SCORE_PIVOT = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_SCORE_CLAW = IdleMode.kBrake;
    public static final IdleMode IDLEMODE_SCORE_AGITATOR = IdleMode.kCoast;
    public static final IdleMode IDELMODE_CLIMB = IdleMode.kBrake;

    public static final boolean INVERT_INTAKE_ELEVATOR = false;
    public static final boolean INVERT_SCORE_ELEVATOR = false;
    public static final boolean INVERT_INTAKE_ROTATE = true;
    public static final boolean INVERT_INTAKE_ROLLER = true;
    public static final boolean INVERT_SCORE_ROTATE = true;
    public static final boolean INVERT_SCORE_PIVOT = false;
    public static final boolean INVERT_SCORE_CLAW = true;
    public static final boolean INVERT_SCORE_AGITATOR = true;
    public static final boolean INVERT_CLIMB_ROTATE = true;

    public static final boolean INVERT_SCORE_PIVOT_ABS = true;
    public static final boolean INVERT_SCORE_ROTATE_ABS = false;
    public static final boolean INVERT_SCORE_CLAW_ABS = false;
    public static final boolean INVERT_INTAKE_ROTATE_ABS = false;

    public static final double RAMPRATE_INTAKE_ELEVATOR = 0.15;
    public static final double RAMPRATE_SCORE_ELEVATOR = 0.275;
    public static final double RAMPRATE_INTAKE_ROTATE = 0.05;
    public static final double RAMPRATE_INTAKE_ROLLER = 0.1;
    public static final double RAMPRATE_SCORE_ROTATE = 0.0;
    public static final double RAMPRATE_SCORE_PIVOT = 0.0;
    public static final double RAMPRATE_SCORE_CLAW = 0.0;
    public static final double RAMPRATE_SCORE_AGITATOR = 0.1;
    public static final double RAMPRATE_CLIMB = 0.1;

    public static final int CURRENTLIMIT_INTAKE_ELEVATOR = 20;
    public static final int CURRENTLIMIT_SCORE_ELEVATOR = 30;
    public static final int CURRENTLIMIT_INTAKE_ROTATE = 20;
    public static final int CURRENTLIMIT_INTAKE_ROLLER = 20;
    public static final int CURRENTLIMIT_SCORE_ROTATE = 30;
    public static final int CURRENTLIMIT_SCORE_PIVOT = 20;
    public static final int CURRENTLIMIT_SCORE_CLAW = 20;
    public static final int CURRENTLIMIT_SCORE_AGITATOR = 20;
    public static final int CURRENTLIMIT_CLIMB_ = 20;

    //Closed Loop controller Config

    public static final double P_INTAKE_ELEVATOR = 0.1;
    public static final double MAX_OUTPUT_INTAKE_ELEVATOR = 0.9;
    public static final double MIN_OUTPUT_INTAKE_ELEVATOR = -0.9;
    public static final double MAX_OUTPUT_STAGE_HANDOFF_INTAKE_ELEVATOR = 0.3;
    public static final double MIN_OUTPUT_STAGE_HANDOFF_INTAKE_ELEVATOR = -0.3;

    public static final double P_SCORE_ELEVATOR = 0.04;
    public static final double MAX_OUTPUT_SCORE_ELEVATOR = 0.9;
    public static final double MIN_OUTPUT_SCORE_ELEVATOR = -0.9;
    public static final double MAX_OUTPUT_STAGE_HANDOFF_SCORE_ELEVATOR = 0.3;
    public static final double MIN_OUTPUT_STAGE_HANDOFF_SCORE_ELEVATOR = -0.3;

    public static final double P_INTAKE_ROTATE = 2.5;
    public static final double MAX_OUTPUT_INTAKE_ROTATE = 0.25;
    public static final double MIN_OUTPUT_INTAKE_ROTATE = -0.25;

    public static final double P_SCORE_ROTATE = 4.0;
    public static final double MAX_OUTPUT_SCORE_ROTATE = 0.35;
    public static final double MIN_OUTPUT_SCORE_ROTATE = -0.35;

    public static final double P_SCORE_PIVOT = 5.0;
    public static final double MAX_OUTPUT_SCORE_PIVOT = 0.1;
    public static final double MIN_OUTPUT_SCORE_PIVOT = -0.1;

    public static final double P_SCORE_CLAW = 3.0;
    public static final double MAX_OUTPUT_SCORE_CLAW = 0.25;
    public static final double MIN_OUTPUT_SCORE_CLAW = -0.25;

    public static final double P_CLIMB = 1.0;
    public static final double MAX_OUTPUT_CLIMB = 1.0;
    public static final double MIN_OUTPUT_CLIMB = -1.0;
    

    //Position Thresholds
    public static final double THRESHOLD_INTAKE_ROTATE_POS = 0.05;
    public static final double THRESHOLD_ELEVATOR_INTAKE_POS = 2.0;
    public static final double THRESHOLD_ELEVATOR_SCORE_POS = 2.0;
    public static final double THRESHOLD_SCORE_ROTATE_POS = 0.05;
    public static final double THRESHOLD_SCORE_PIVOT_POS = 0.05;
    public static final double THRESHOLD_SCORE_CLAW_POS = 0.05;
    public static final double THRESHOLD_CLIMB_POS = 0.05;

    //LaserCan Thresholds

    public static final double THRESHOLD_LASERCAN_INTAKE_LEFT = 50; //In mm
    public static final double THRESHOLD_LASERCAN_INTAKE_RIGHT = 50; //In mm

    //LED Constants

    public static final int LED_ID = 1;
    public static final int LED_SIZE = 30;

    public static class SCORING_POSES{
        public class CENTER_NEAR{
            public static final double X = 3.34;
            public static final double Y = 4.03;
            public static final double T = 0.0;
        }

        public static final class CENTER_FAR{
            public static final double X = 5.64;
            public static final double Y = 4.03;
            public static final double T = 180.0;
        }

        public static final class LEFT_NEAR{
            public static final double X = 3.91;
            public static final double Y = 5.02;
            public static final double T = -60.0;
        }

        public static final class LEFT_FAR{
            public static final double X = 5.07;
            public static final double Y = 5.01;
            public static final double T = -120.0;
        }

        public static final class RIGHT_NEAR{
            public static final double X = 3.92;
            public static final double Y = 3.02;
            public static final double T = 60.0;
        }

        public static final class RIGHT_FAR{
            public static final double X = 5.03;
            public static final double Y = 3.05;
            public static final double T = 120.0;
        }
    };

    public static class LOADING_POSES{
        public static final class LEFT{
            public static final double X = 1.654;
            public static final double Y = 7.436;
            public static final double T = 120.0;
        }

        public static final class RIGHT{
            public static final double X = 1.67;
            public static final double Y = 0.567;
            public static final double T = -120.0;
        }
    }
}
