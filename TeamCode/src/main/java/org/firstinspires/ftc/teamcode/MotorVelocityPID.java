package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp
public class MotorVelocityPID extends CommandOpMode {

    PIDFController pidf = new PIDFController(0.5, 0, 0, 0);
    MotorEx m_flywheel = new MotorEx(hardwareMap, "flywheel");
    DcMotor flywheel = m_flywheel.motorEx;
    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }

    double setpoint = 0;

    @Override
    public void initialize() {
        m_flywheel.setRunMode(Motor.RunMode.VelocityControl);

        m_flywheel.setVeloCoefficients(0.05, 0.01, 0.31);
        double[] coeffs = m_flywheel.getVeloCoefficients();
        double kP = coeffs[0];
        double kI = coeffs[1];
        double kD = coeffs[2];

        m_flywheel.setFeedforwardCoefficients(0.92, 0.47);
        double[] ffCoeffs = m_flywheel.getFeedforwardCoefficients();
        double kS = ffCoeffs[0];
        double kV = ffCoeffs[1];

        m_flywheel.setFeedforwardCoefficients(0.92, 0.47, 0.3);
        ffCoeffs = m_flywheel.getFeedforwardCoefficients();
        double kA = ffCoeffs[2];

        double output = pidf.calculate(
                m_flywheel.getVelocity(), setpoint
        );
        pidf.setSetPoint(setpoint);
    }

    @Override
    public void run () {
        if (gamepad1.right_bumper) {
            setpoint = setpoint + 0.1;
            if (setpoint > 1) {
                setpoint = 1;
            }
        }

        if (gamepad1.right_bumper) {
            setpoint = setpoint - 0.1;
            if (setpoint < -1) {
                setpoint = -1;
            }
        }

        while (!pidf.atSetPoint()) {
            double output = pidf.calculate(
                    m_flywheel.getVelocity()  // the measured value
            );
            m_flywheel.setVelocity(output);
        }
        m_flywheel.stopMotor(); // stop the motor

// NOTE: motors have internal PID control
    }
}
