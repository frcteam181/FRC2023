package frc.robot;

public final class Constants {

    /* Drive Base Gains */

    public static final int k_DRIVE_SLOT_ID = 0;
    public static final int k_TURN_SLOT_ID = 1;
    public static final Gains k_DriveGains = new Gains(0, 0, 0, 0, 0, 0, 0);
    public static final Gains k_TurnGains = new Gains(0, 0, 0, 0, 0, 0, 0);

    /* Controller Ports */

    public static final int k_DRIVER_CONTROLLER = 0;

    /* Controller Deadbands */

    public static final double k_DriverDeadband = 0.2;

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/1yODdikPXsuiSYJ6ICP2WU1t-p1ZJTm6qR6IG0DP6S7M/edit#gid=0

    /* CAN IDs */

    public static final int k_PDH = 1;
    public static final int k_PH = 2;
    public static final int k_LEFT_LEADER = 4;
    public static final int k_LEFT_FOLLOWER = 3;
    public static final int k_RIGHT_LEADER = 6;
    public static final int k_RIGHT_FOLLOWER = 5;
    public static final int k_PIGEON = 7;

}