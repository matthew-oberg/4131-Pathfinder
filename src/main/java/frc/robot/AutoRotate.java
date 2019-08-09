package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class AutoRotate implements PIDOutput {

    private double pTurn = 0.012;
    private double iTurn = 0.0;
    private double dTurn = 0.0078;
    private double fTurn = 0.0;

    private final double toleranceDegrees = 2.0;

    private double turnRate;

    PIDController turn = new PIDController(pTurn, iTurn, dTurn, fTurn, Robot.ahrs, this);

    public AutoRotate() {

    }

    public PIDController getTurn() {
        return turn;
    }

    public double getTolerance() {
        return toleranceDegrees;
    }

    public double getTurnRate() {
        return turnRate;
    }

    @Override
    public void pidWrite(double output) {
        if (turn.onTarget()) {
            turnRate = 0;
        } else {
            turnRate = output;
        }
    }
}