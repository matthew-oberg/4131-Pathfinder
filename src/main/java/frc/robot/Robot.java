/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot implements PIDOutput {

    private static double pTurn = 0.0075;
    private static double iTurn = 0.0;
    private static double dTurn = 0.0;
    private static double fTurn = 0.0;

    private static final double toleranceDegrees = 2.0;

    private static int turnTarget = 0;

    private static final double pDrive = 0.0;
    private static final double iDrive = 0.0;
    private static final double dDrive = 0.0;
    private static final double fDrive = 0.0;

    private static final double toleranceTicks = 25.0;

    private static double turnRate;

    private static int driveTarget = 0;

    private static double driveRate;

    TalonSRX leftOne = new TalonSRX(1);
    TalonSRX leftTwo = new TalonSRX(2);
    TalonSRX rightOne = new TalonSRX(3);
    TalonSRX rightTwo = new TalonSRX(4);

    XboxController controller = new XboxController(0);
    GenericHID.Hand leftHand = GenericHID.Hand.kLeft;
    GenericHID.Hand rightHand = GenericHID.Hand.kRight;

    AHRS ahrs = new AHRS(SPI.Port.kMXP);

    PIDController turn = new PIDController(pTurn, iTurn, dTurn, fTurn, ahrs, this);
    //PIDController drive = new PIDController(pDrive, iDrive, dDrive, dDrive, leftOne.getSelectedSensorPosition(), this);

    @Override
    public void robotInit() {
        turn.setInputRange(0.0, 360.0);
        turn.setOutputRange(-1, 1);
        turn.setAbsoluteTolerance(toleranceDegrees);
        turn.setContinuous(true);
        turn.enable();
        turn.setSetpoint(0);
    
    /*drive.setInputRange(-254000.0, 254000.0);
    drive.setOutputRange(-1, 1);
    drive.setAbsoluteTolerance(toleranceTicks);
    drive.setContinuous(true);
    drive.enable();
    drive.setSetpoint(0);*/
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        leftOne.setSelectedSensorPosition(0);
        rightOne.setSelectedSensorPosition(0);

        ahrs.reset();
    }

    @Override
    public void teleopPeriodic() {
        if (controller.getAButtonReleased()) {
            reset();
        }

        if (controller.getPOV() == 0) {
            setTarget(0);
        } else if (controller.getPOV() == 90) {
            setTarget(90);
        } else if (controller.getPOV() == 180) {
            setTarget(180);
        } else if (controller.getPOV() == 270) {
            setTarget(270);
        } else {}

        if (turn.onTarget()) {
            standardDrive();
        } else {
            turn.setSetpoint(turnTarget);
            rotate();
        }

        dashboard();
    }

    public void standardDrive() {
        double throttle = 0;
        double turn = 0;

        if (controller.getY(leftHand) > 0.05 || controller.getY(leftHand) < -0.05)
            throttle = controller.getY(leftHand);

        if (controller.getX(rightHand) > 0.05 || controller.getX(rightHand) < -0.05)
            turn = controller.getX(rightHand);

        leftOne.set(ControlMode.PercentOutput, turn - throttle);
        leftTwo.set(ControlMode.PercentOutput, turn - throttle);

        rightOne.set(ControlMode.PercentOutput, turn + throttle);
        rightTwo.set(ControlMode.PercentOutput, turn + throttle);
    }

    public void reset() {
        ahrs.reset();
    }

    @Override
    public void pidWrite(double output) {
        if (turn.onTarget()) {
            turnRate = 0;
        } else {
            turnRate = output;
        }
    }

    public void setTarget(int target) {
        if (target > 360 || target < 0) {
            return;
        } else {
            turnTarget = target;
        }
    }

    public void rotate() {
        leftOne.set(ControlMode.PercentOutput, turnRate);
        leftTwo.set(ControlMode.PercentOutput, turnRate);

        rightOne.set(ControlMode.PercentOutput, turnRate);
        rightTwo.set(ControlMode.PercentOutput, turnRate);
    }

    public void dashboard() {
        SmartDashboard.putNumber("POV", controller.getPOV());
        SmartDashboard.putNumber("Setpoint", turn.getSetpoint());
        SmartDashboard.putNumber("Angle", ahrs.getAngle());
        SmartDashboard.putNumber("Target", turnTarget);
        SmartDashboard.putBoolean("On Target", turn.onTarget());
        SmartDashboard.putNumber("p", turn.getP());
        SmartDashboard.putNumber("i", turn.getI());
        SmartDashboard.putNumber("d", turn.getD());
        SmartDashboard.putNumber("turnrate", turnRate);
        /*turn.setP(p);
        turn.setI(i);
        turn.setD(d);*/
    }
}