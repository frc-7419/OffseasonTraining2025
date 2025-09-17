package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants.ArmConstants;

/**
 * The {@code ArmSubsystem} class controls the wrist subsystem of the robot. It manages the motion
 * and angle of the elevator using TalonFX motors with a fused CANCoder.
 */
public class ArmSubsystem extends SubsystemBase {
  private enum ControlMode {
    MANUAL,
    MOTIONMAGIC
  }

  private final TalonFX wristMotor = new TalonFX(ArmConstants.kArmMotorID);

  private final CANcoder wristEncoder = new CANcoder(ArmConstants.kArmEncoderID);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final MotionMagicExpoVoltage motionMagicRequest =
      new MotionMagicExpoVoltage(0).withSlot(0);

  private ControlMode controlMode = ControlMode.MANUAL;

  /** Creates a new {@code ArmSubsystem} with a TalonFX and a CANcoder. */
  public ArmSubsystem() {
    wristMotor.getConfigurator().apply(ArmConstants.kArmTalonFXConfiguration);
    wristEncoder.getConfigurator().apply(ArmConstants.kArmCANCoderConfig);
    brake();
  }

  /**
   * Sets the wrist power in manual mode.
   *
   * @param power The power to set, ranging from -1 to 1. Positive values move the elevator up, and
   *     negative values move it down.
   */
  public void setPower(double power) {
    if (controlMode == ControlMode.MOTIONMAGIC) {
      return;
    }

    power = Math.max(-1, Math.min(1, power));

    wristMotor.setControl(
        velocityRequest
            .withVelocity(power * ArmConstants.kMaxSpeed.in(RotationsPerSecond))
            .withLimitForwardMotion(getPosition().gte(ArmConstants.kMaxAngle))
            .withLimitReverseMotion(getPosition().lte(ArmConstants.kMinAngle)));
  }

  /**
   * Returns a Command that drives the wrist to a specific angle and then ends, returning the
   * control mode to MANUAL.
   *
   * @param angle The target angle
   * @return A command for scheduling.
   */
  public Command setAngle(Angle angle) {
    return this.runEnd(() -> toAngle(angle), () -> switchControlMode(ControlMode.MANUAL));
  }

  /**
   * Returns a Command that applies manual wrist control (ex, from a joystick).
   *
   * @param power The power from -1 to 1.
   * @return A command for scheduling.
   */
  public Command joystickControl(CommandXboxController joystick) {
    return this.run(() -> setPower(joystick.getRightY()));
  }

  /** Sets the TalonFX to brake mode (resists motion when no power is applied). */
  public void brake() {
    wristMotor.set(0);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /** Sets the TalonFX to coast mode (spins freely when no power is applied). */
  public void coast() {
    wristMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public Angle getPosition() {
    return wristEncoder.getAbsolutePosition().getValue();
  }

  public AngularVelocity getVelocity() {
    return wristEncoder.getVelocity().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle (Rotations)", getPosition().in(Rotations));
    SmartDashboard.putNumber("Arm Angle (Degrees)", getPosition().in(Degrees));
    SmartDashboard.putNumber(
        "Arm Velocity (RotationsPerSecond)", getVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Arm Temperature (Celsius)", wristMotor.getDeviceTemp().getValue().in(Celsius));
  }

  /**
   * Moves the wrist to a specified angle using Motion Magic.
   *
   * @param angle The desired angle (e.g., 0° is horizontal, 90° is vertical).
   */
  private void toAngle(Angle angle) {
    controlMode = ControlMode.MOTIONMAGIC;

    wristMotor.setControl(
        motionMagicRequest
            .withPosition(angle.in(Rotations))
            .withLimitForwardMotion(getPosition().gte(ArmConstants.kMaxAngle))
            .withLimitReverseMotion(getPosition().lte(ArmConstants.kMinAngle)));
  }

  /** Switches the current control mode. */
  private void switchControlMode(ControlMode mode) {
    this.controlMode = mode;
  }

public void runMotor(double power) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runMotor'");
}
}
