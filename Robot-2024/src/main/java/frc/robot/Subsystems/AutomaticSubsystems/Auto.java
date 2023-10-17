package frc.robot.Subsystems.AutomaticSubsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Subsystems.Cameras.Limelight;

public class Auto {
    private int mode, state;
    private double forward, strafe, rotation, colorTimeelapsed, minY, oldY, startPos;
    private boolean balancing, scoringhigh;
    private Timer timer;

    protected SwerveDriveRobot swerve;
    protected Intake intake;
    protected RotateArm rotateArm;
    protected Elevator horizontalElevator;
    protected Elevator verticalElevator;
    protected Limelight limelight;
    protected Gyro gyro;

    public Auto(SwerveDriveRobot s, Intake i, RotateArm r, Elevator he, Elevator ve, Limelight l, Gyro g) {
        swerve = s;
        intake = i;
        rotateArm = r;
        horizontalElevator = he;
        verticalElevator = ve;
        limelight = l;
        gyro = g;

        state = 0;
        colorTimeelapsed = 0;
        minY = 0;
        oldY = 100;
        startPos = swerve.getSwervePosition();
        balancing = false;
        scoringhigh = false;
        timer = new Timer();
    }

    public void setMode(int m) {
        mode = m;
    }

    public void initiateAuto() {
        forward = 0;
        strafe = 0;
        rotation = 0;
        timer.start();
        swerve.setSpeedFactor(1);
    }

    public void periodicAuto() {
        if(mode == BALANCE) {
            balanceAuto();
        } else if(mode == SCORE) {
            scoreAuto();
        } else if(mode == BACKUP) {
            backUpAuto();
        }
    }

    public void changeState() {
        timer.stop();
        timer.reset();
        timer.start();
        state++;
    }

    public void pickup() {  
        //function recalled whever the robot needs to pickup the game material
        //can be used for teleop
        /* 
        vertical elevator up/down y feet
        horizontal elevator forward/backwards x feet
        arm rotate about z degrees forward
        pickup game material using motors
        */
    }

    public void place() { 
        //function recalled whever the robot needs to drop/place the game material
        //can be used for teleop
        /* 
        vertical elevator up/down y feet
        horizontal elevator forward/backwards x feet
        arm rotate about z degrees forward/backwards
        release game material using motors
        wait .5 seconds
        arm rorate about z degrees forward/backwards
        */
    }

    public void balanceAuto() {  //scores the pre-loaded game material and then balances
        switch(state) {
            /*case 0: alignAprilTag();
                break;
            case 1: scoreLow();
                break;
            case 2: driveBack();
                break;*/
            case 0: balance();
                break;
            default: swerve.zeroMotors();
        }
    }

    public void scoreAuto() {  //only scores
        switch(state) {
            case 0: scoreHigh();
                break;
            case 1: driveBack();
                break;
            case 2: turnAround();
                break;
            case 3: alignCube();
                break;
            case 4: turnToAprilTag();
                break;
            case 5: alignAprilTag();
                break;
            case 6: scoreLow();
                break;
            default: swerve.zeroMotors();
        }
    }

    public void backUpAuto() {
        switch(state) {
            case 0: scoreHigh();
                break;
            case 1: driveBack();
                break;
            default: swerve.zeroMotors();
        }
    }

    public void scoreBalanceAuto() {  //scores the pre-loaded game material and another game material, then balances
        /* 
        drive forward to score zone using april tags until 1 feet away from april tags
        align with reflective tape
        intake.place(x,y,z)
        reset motors
        rotate robot 180 degrees 
        robot drive forward to get game material 
        intake.pickup(x,y,z)
        rotate robot 180 degrees
        drive forward to score zone using april tags until 1 feet away from april tags
        align with reflective tape
        intake.place(x,y,z)
        reset motors
        drive left/right until parellel to center (in front) of charging station
        drive forward/backward x feet to the chargin station
        brake the motors/wheels
         */
    }

    public void intakePiece() {
        intake.wheelsIn();
        intake.close();
    }

    private void alignAprilTag() {
        if(Limelight.getPipeline() != 0) {
            Limelight.setPipeline(0);
        }
        else {
            if(/*Timer.getMatchTime() < 14.0 &&*/ timer.get() < 3) {
                forward = ((Limelight.getArea() > 1.5) ? -0.5 : ((Limelight.getArea() < 0.5 && Limelight.getArea() > 0) ? Math.min(1, 1/(2 * Limelight.getArea())) : 0));
                strafe = (Math.abs(Limelight.getX()) > 3 && Limelight.getArea() < 2) ? Limelight.getX()/20 : 0;
                rotation = 0;//(Limelight.getArea() > 0.5 && Math.abs(Limelight.getTargetYaw()) > 0.5) ? strafe : 0;
                swerve.drive(forward, strafe, rotation);
                if((forward == 0 && Math.abs(strafe) < 0.1)){
                    swerve.zeroMotors();
                    if(Limelight.getX() != 0) {
                        changeState();
                    }
                }
            }
            else {
                swerve.zeroMotors();
                changeState();
            }
        }
    }

    private void alignCube() {
        if(Limelight.getPipeline() != 1) {
            Limelight.setPipeline(1);
        }
        else {
            if(oldY == 100 && Limelight.getArea() > 0.1) {
                oldY = Limelight.getY();
            }
            if(minY > -13) {
                if(timer.get() - colorTimeelapsed > 0.125 && Math.abs(oldY - Limelight.getY()) < 3) {
                    forward = (Limelight.getArea() > 6) ? -0.5 : ((Limelight.getArea() > 0.2) ? Math.min(0.6, 0.6/(Limelight.getArea())) : 0);
                    strafe = (Limelight.getX() < -5 || Limelight.getX() > 5) ? Limelight.getX()/55 : 0;
                    rotation = (Limelight.getArea() > 0.5) ? strafe/2 : 0;
                    colorTimeelapsed = timer.get();
                    oldY = Limelight.getY();
                }
                swerve.drive(forward, strafe, rotation);
            }
            else if(minY <= -13 && (timer.get() - colorTimeelapsed) < 0.45) {
                swerve.drive(0.85, 0.25, 0);
                intake.wheelsIn();
            } 
            else {
                startPos = swerve.getSwervePosition();
                swerve.zeroMotors();
                intake.close();
                changeState();
            }
            
            SmartDashboard.putNumber("minY", minY);
            if(Limelight.getY() < minY) {
                minY = Limelight.getY();
            }
        }
    }

    private void scoreLow() {
        if(timer.get() < 2) {
            intake.wheelsOut();
        } else {
            intake.zero();
            changeState();
        }
    }

    public void scoreHigh() {
        if((timer.get() < 0.1) || (timer.get() < 0.8 && rotateArm.getArmMovingOut())){
            scoringhigh = true;
            rotateArm.extend(1);
        }
        else if((timer.get() > 2.5 && timer.get() < 2.6) || (rotateArm.getArmMovingIn())){
            rotateArm.retract(1);
        }
        else {
            rotateArm.zero();
        }

        if((timer.get() < 0.1) || (timer.get() < 2 && horizontalElevator.getElevatorMovingOut())){
            horizontalElevator.movePositive();
        }
        else if((timer.get() > 2.5 && timer.get() < 2.6) || (horizontalElevator.getElevatorMovingIn())){
            horizontalElevator.moveNegative();
        }
        else {
            horizontalElevator.zero();
        }

        if((timer.get() < 0.1) || (timer.get() < 2 && verticalElevator.getElevatorMovingOut())){
            verticalElevator.movePositive();
        }
        else if((timer.get() > 2.5 && timer.get() < 2.6) || (verticalElevator.getElevatorMovingIn())){
            verticalElevator.moveNegative();
        }
        else {
            verticalElevator.zero();
        }

        if(timer.get() > 2 && timer.get() < 2.5) {
            intake.wheelsOut();
        }
        else {
            intake.zero();
        }
        
        if(timer.get() > 2.6 && !horizontalElevator.getElevatorMovingIn() && !verticalElevator.getElevatorMovingIn() && !rotateArm.getArmMovingIn()) {
            scoringhigh = false;
            rotateArm.zero();
            verticalElevator.zero();
            horizontalElevator.zero();
            intake.zero();
            changeState();
        }
    }

    private void driveBack() {
        if(timer.get() < 0.6 || (timer.get() < 2 && rotateArm.getArmMovingIn())){
            rotateArm.retract(1);
        }
        else {
            rotateArm.zero();
        }
        if(timer.get() < 1.65) {
            swerve.drive(-1, 0.1, 0);
        } else {
            swerve.zeroMotors();
            startPos = swerve.getSwervePosition();
            changeState();
        }
    }

    private void balance() {
        if(gyro.getRollValue() > 10 && !balancing) {
            swerve.setMode(NeutralMode.Brake);
            balancing = true;
            timer.stop();
            timer.reset();
            timer.start();
        }
        if(!balancing) {
            swerve.drive(-1, 0, 0);
        }
        else {
            if(timer.get() < 2) {
                swerve.drive(-0.75, 0, 0);
            }
            else if(gyro.getRollValue() > 8) {
                swerve.drive(-0.5, 0, 0);
            }
            else if(gyro.getRollValue() > 5) {
                swerve.drive(-0.4, 0, 0);
            }
            else if(gyro.getRollValue() > 4) {
                swerve.drive(-0.15, 0, 0);
            }
            else if(gyro.getRollValue() > 2) {
                swerve.drive(-0.05, 0, 0);
            }
            else if(gyro.getRollValue() < -1) {
                swerve.drive(0.5, 0, 0);
            }
            else {
                swerve.zeroMotors();
                changeState();
            }
        }
    }

    private void turnAround() { //add in turn left/right cases based on AprilTag ID
        if((timer.get() < 0.1) || (timer.get() < 2 && rotateArm.getArmMovingOut())){
            Limelight.setPipeline(1);
            rotateArm.extend(1);
            intake.open();
        } else {
            rotateArm.zero();
        }
        if(Math.abs(swerve.getSwervePosition() - startPos) < 62000) {
            swerve.drive(0, 0, 1);
        } else {
            swerve.zeroMotors();
        }
        if(Math.abs(swerve.getSwervePosition() - startPos) >= 62000 && !rotateArm.getArmMovingOut()) {
            changeState();
        }
    }

    private void turnToAprilTag() {
        if(Limelight.getPipeline() != 0) {
            Limelight.setPipeline(0);
            startPos = swerve.getSwervePosition();
            timer.stop();
        } else {
            timer.start();
            if(timer.get() > 0.5) {
                intake.zero();
                if((Math.abs(swerve.getSwervePosition() - startPos) > 60000)) {
                    if(Limelight.getArea() > 0.035 && Limelight.getX() > -5) {
                        changeState();
                        swerve.zeroMotors();
                    } else {
                        swerve.drive(0, 0, 0.4);
                    }
                } else {
                    swerve.drive(0, 0, 0.8);
                }
                
            }
        }
    }

    // private void alignLeft() {
    //     /*if(Limelight.getArea() > 0.03 && Limelight.getX() < 23) {
    //         swerve.drive(0, -0.75, 0);
    //     } else {
    //         swerve.zeroMotors();
    //         changeState();
    //     }*/
    // }

    public boolean getScoringHigh() {
        return scoringhigh;
    }
}