/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    static TalonSRX leftOne = new TalonSRX(1);
    static TalonSRX leftTwo = new TalonSRX(2);
    static TalonSRX rightOne = new TalonSRX(3);
    static TalonSRX rightTwo = new TalonSRX(4);

    XboxController controller = new XboxController(0);
    GenericHID.Hand leftHand = GenericHID.Hand.kLeft;
    GenericHID.Hand rightHand = GenericHID.Hand.kRight;

    static AHRS ahrs = new AHRS(SPI.Port.kMXP);

    AutoRotate turn = new AutoRotate();
    AutoDrive drive = new AutoDrive();
    AutoAlign align = new AutoAlign();
    AutoRange range = new AutoRange();

    static double tv, tx, ty, ta, tvert, tl;
    static double tx_prev = 0;

    @Override
    public void robotInit() {
        turn.getTurn().setInputRange(-180.0, 180.0);
        turn.getTurn().setOutputRange(-1, 1);
        turn.getTurn().setAbsoluteTolerance(turn.getTolerance());
        turn.getTurn().setContinuous(true);
        turn.getTurn().enable();
        turn.getTurn().setSetpoint(0);
    
        drive.getDrive().setInputRange(Double.MIN_VALUE, Double.MAX_VALUE);
        drive.getDrive().setOutputRange(-1, 1);
        drive.getDrive().setAbsoluteTolerance(drive.getTolerance());
        drive.getDrive().setContinuous(true);
        drive.getDrive().enable();
        drive.getDrive().setSetpoint(0);

        align.getAlign().setInputRange(-30.0, 30.0);
        align.getAlign().setOutputRange(-1, 1);
        align.getAlign().setAbsoluteTolerance(turn.getTolerance());
        align.getAlign().setContinuous(true);
        align.getAlign().enable();
        align.getAlign().setSetpoint(0);

        range.getRange().setInputRange(0, 100);
        range.getRange().setOutputRange(-1, 1);
        range.getRange().setAbsoluteTolerance(range.getTolerance());
        range.getRange().setContinuous(true);
        range.getRange().enable();
        range.getRange().setSetpoint(0);
    }

    @Override
    public void teleopInit() {
        leftOne.setNeutralMode(NeutralMode.Brake);
        leftTwo.setNeutralMode(NeutralMode.Brake);
        rightOne.setNeutralMode(NeutralMode.Brake);
        rightTwo.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void teleopPeriodic() {
        if (tx != 0) {
            tx_prev = tx;
        }

        if (controller.getAButtonReleased()) {
            resetTurn();
        }

        if (controller.getXButtonReleased()) {
            resetDrive();
        }

        if (controller.getPOV() == 0) {
            setTargetTurn(0);
        } else if (controller.getPOV() == 90) {
            setTargetTurn(90);
        } else if (controller.getPOV() == 180) {
            setTargetTurn(180);
        } else if (controller.getPOV() == 270) {
            setTargetTurn(-90);
        } else {}

        if (controller.getBButton()) {
            autoTurn();
        } else if (controller.getYButton()) {
            autoDrive();
        } else if (controller.getBackButton()) {
            if (tv == 0)
                autoSeek();
            else
                autoAlign();
        } else if (controller.getStartButton()) {
            if (tv == 0)
                autoSeek();
            else
                autoRange();
        } else if (controller.getBumper(rightHand)) {
            if (tv == 0)
                autoSeek();
            else
                autoChase();
        } else if (controller.getBumper(leftHand)) {
            fieldCentricDrive();
        } else {
            standardDrive();
        }

        limelightVals();

        dashboard();
    }

    public void limelightVals() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
        tl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
    }

    public void dashboard() {
        SmartDashboard.putNumber("Turn Setpoint", turn.getTurn().getSetpoint());
        SmartDashboard.putNumber("Drive Setpoint", drive.getDrive().getSetpoint());

        SmartDashboard.putBoolean("Turn on Target", turn.getTurn().onTarget());
        SmartDashboard.putBoolean("Drive on Target", drive.getDrive().onTarget());

        SmartDashboard.putNumber("Turn Rate", turn.getTurnRate());
        SmartDashboard.putNumber("Drive Rate", drive.getDriveRate());

        SmartDashboard.putNumber("Angle", ahrs.getAngle());
        SmartDashboard.putNumber("Ticks", leftOne.getSelectedSensorPosition());

        SmartDashboard.putBoolean("Align on Target", align.getAlign().onTarget());
        SmartDashboard.putNumber("Align Rate", align.getTurnRate());

        SmartDashboard.putBoolean("Range on Target", range.getRange().onTarget());
        SmartDashboard.putNumber("Range Rate", range.getDriveRate());

        SmartDashboard.putData("Turn PID", turn.getTurn());
        SmartDashboard.putData("Drive PID", drive.getDrive());
        SmartDashboard.putData("Align PID", align.getAlign());
        SmartDashboard.putData("Range PID", range.getRange());

        SmartDashboard.putNumber("Valid Targets (tv)", tv);
        SmartDashboard.putNumber("Horizontal Offset (tx)", tx);
        SmartDashboard.putNumber("Vertical Offset (ty)", ty);
        SmartDashboard.putNumber("Target Area % (ta)", ta);
        SmartDashboard.putNumber("Box Height (tvert)", tvert);
        SmartDashboard.putNumber("Latency (tl)", tl);
    }

    public void fieldCentricDrive() {
        double throttle = 0;
        double rotate = turn.getTurnRate();

        if (controller.getY(leftHand) > 0.05 || controller.getY(leftHand) < -0.05)
            throttle = controller.getY(leftHand);

        leftOne.set(ControlMode.PercentOutput, rotate + throttle);
        leftTwo.set(ControlMode.PercentOutput, rotate + throttle);

        rightOne.set(ControlMode.PercentOutput, rotate - throttle);
        rightTwo.set(ControlMode.PercentOutput, rotate - throttle);
    }

    public void standardDrive() {
        double throttle = 0;
        double rotate = 0;

        if (controller.getY(leftHand) > 0.05 || controller.getY(leftHand) < -0.05)
            throttle = controller.getY(leftHand);

        if (controller.getX(rightHand) > 0.075 || controller.getX(rightHand) < -0.075)
            rotate = controller.getX(rightHand);

        leftOne.set(ControlMode.PercentOutput, rotate + throttle);
        leftTwo.set(ControlMode.PercentOutput, rotate + throttle);

        rightOne.set(ControlMode.PercentOutput, rotate - throttle);
        rightTwo.set(ControlMode.PercentOutput, rotate - throttle);
    }

    public void resetTurn() {
        ahrs.reset();
    }

    public void setTargetTurn(int target) {
        if (target >= -180 && target <= 180) {
            turn.getTurn().setSetpoint(target);
        } else {
            return;
        }
    }

    public void autoTurn() {
        leftOne.set(ControlMode.PercentOutput, turn.getTurnRate());
        leftTwo.set(ControlMode.PercentOutput, turn.getTurnRate());

        rightOne.set(ControlMode.PercentOutput, turn.getTurnRate());
        rightTwo.set(ControlMode.PercentOutput, turn.getTurnRate());
    }

    public void resetDrive() {
        leftOne.setSelectedSensorPosition(0);
        rightOne.setSelectedSensorPosition(0);
    }

    public void setTargetDrive(int target) {
        drive.getDrive().setSetpoint(target);
    }

    public void autoDrive() {
        leftOne.set(ControlMode.PercentOutput, drive.getDriveRate());
        leftTwo.set(ControlMode.PercentOutput, drive.getDriveRate());

        rightOne.set(ControlMode.PercentOutput, -drive.getDriveRate());
        rightTwo.set(ControlMode.PercentOutput, -drive.getDriveRate());
    }

    public void autoAlign() {
        leftOne.set(ControlMode.PercentOutput, -align.getTurnRate());
        leftTwo.set(ControlMode.PercentOutput, -align.getTurnRate());

        rightOne.set(ControlMode.PercentOutput, -align.getTurnRate());
        rightTwo.set(ControlMode.PercentOutput, -align.getTurnRate());
    }

    public void autoRange() {
        leftOne.set(ControlMode.PercentOutput, -range.getDriveRate());
        leftTwo.set(ControlMode.PercentOutput, -range.getDriveRate());

        rightOne.set(ControlMode.PercentOutput, range.getDriveRate());
        rightTwo.set(ControlMode.PercentOutput, range.getDriveRate());
    }

    public void autoChase() {
        double throttle = 0;
        double rotate = 0;

        throttle = -range.getDriveRate();
        rotate = -align.getTurnRate();

        leftOne.set(ControlMode.PercentOutput, rotate + throttle);
        leftTwo.set(ControlMode.PercentOutput, rotate + throttle);

        rightOne.set(ControlMode.PercentOutput, rotate - throttle);
        rightTwo.set(ControlMode.PercentOutput, rotate - throttle);
    }

    public void autoSeek() {
        if (tx_prev > 0) {
            leftOne.set(ControlMode.PercentOutput, 0.2);
            leftTwo.set(ControlMode.PercentOutput, 0.2);

            rightOne.set(ControlMode.PercentOutput, 0.2);
            rightTwo.set(ControlMode.PercentOutput, 0.2);
        } else if (tx_prev < 0) {
            leftOne.set(ControlMode.PercentOutput, -0.2);
            leftTwo.set(ControlMode.PercentOutput, -0.2);

            rightOne.set(ControlMode.PercentOutput, -0.2);
            rightTwo.set(ControlMode.PercentOutput, -0.2);
        }
    }
}