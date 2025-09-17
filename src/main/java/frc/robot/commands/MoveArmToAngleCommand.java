package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.lang.Math;

public class MoveArmToAngleCommand extends CommandBase {
  private final ArmSubsystem m_arm;
  private final Angle m_setpoint;
  private final double TOLERANCE = 2.0;

  public MoveArmToAngleCommand(ArmSubsystem arm, Angle setpoint) {
    m_arm = arm;
    m_setpoint = setpoint;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    System.out.println("Moving arm to setpoint: " + m_setpoint);
  }

  @Override
  public void execute() {
    double currentAngle = m_arm.getPosition().in(Degrees);
    if (currentAngle < m_setpoint.in(Degrees) - TOLERANCE) {
      m_arm.runMotor(0.5); // Fixed power for simple control
    } else if (currentAngle > m_setpoint.in(Degrees) + TOLERANCE) {
      m_arm.runMotor(-0.5); // Fixed power for simple control
    } else {
      m_arm.runMotor(0);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getPosition().in(Degrees) - m_setpoint.in(Degrees)) <= TOLERANCE;
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.runMotor(0);
    System.out.println("MoveArmToAngle finished. Final position: " + m_arm.getPosition());
  }
}

}





