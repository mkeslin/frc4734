package frc.robot;

public class Constants {
    public static final int FRDID = 1; //Front right drive ID
    public static final int FLDID = 2; //Front left drive ID
    public static final int BRDID = 3; //Back right drive ID
    public static final int BLDID = 4; //Back left drive ID
    public static final int FRSID = 5; //Front right steer ID
    public static final int FLSID = 6; //Front left steer ID
    public static final int BRSID = 7; //Back right steer ID
    public static final int BLSID = 8; //Back left steer ID
    public static final int FREID = 11; //Front right encoder ID
    public static final int FLEID = 12; //Front left encoder ID
    public static final int BREID = 9; //Back right encoder ID
    public static final int BLEID = 10; //Back left encoder ID
    public static final int XC1ID = 0; //Xbox controller 1 ID
    public static final int XC2ID = 1; //Xbox controller 2 ID

    public static final double FREO = -36+30-15-98+90; //Front right encoder offset
    public static final double FLEO = -55+40+45+15+105+90-180-15; //Front left encoder offset
    public static final double BREO = -60+35+110-180+22+5; //back right encoder offset 38
    public static final double BLEO = -176-93+70; //back left encoder offset

    public static final int HORELEVATOR1ID = 13;
    public static final int HORELEVATOR2ID = 14;
    public static final int VERTELEVATOR1ID = 15;
    public static final int VERTELEVATOR2ID = 16;
    public static final int ROTATEARMID = 17;
    public static final int INTAKELEFTID = 18;
    public static final int INTAKERIGHTID = 19;
    public static final int PNEUMATICHUBID = 20;
    
    public static final int CRX = 4; //Right X
    public static final int CRY = 5; //Right Y
    public static final int CRT = 3; //Right Trigger
    public static final int CRB = 6; //Right Bumper
    public static final int CLX = 0; //Left X
    public static final int CLY = 1; //Left Y
    public static final int CLT = 2; //Left Trigger
    public static final int CLB = 5; //Left Bumper
    public static final int CB = 7; //Back
    public static final int CS = 8; //Start
    public static final int CAB = 1; //A
    public static final int CBB = 2; //B
    public static final int CXB = 3; //X
    public static final int CYB = 4; //Y
    
    public static final double WBL = 0.58; //Wheelbase length 19.5in 0.50m
    public static final double WBW = 0.50; //Wheelbase width 23.0in 0.58m
    public static final double WBR = Math.sqrt(Math.pow(WBL, 2) + Math.pow(WBW, 2)); //Wheelbase diameter
    public static final double DTS = 0.5; //Drivetrain speed 0.5

    public static final int BALANCE = 0;
    public static final int SCORE = 1;
    public static final int BACKUP = 2;
}