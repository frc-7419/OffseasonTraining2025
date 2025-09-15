package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.*;

public class Constants {
  public static class RobotConstants {
    public static final String kCANivoreBus = "7419";
  }

  public static class ArmConstants {
    public static final int kArmMotorID = 0; // Arbitrary ID (change)
    public static final int kArmEncoderID = 0; // Arbitrary ID (change)
    public static final AngularVelocity kMaxSpeed =
        RotationsPerSecond.of(1); // Arbitrary velocity (change)
    public static final Angle kAngleTolerance = Degrees.of(5); // Arbitrary angle (change)
    public static final Angle kMaxAngle = Degrees.of(90); // Arbitrary angle (change)
    public static final Angle kMinAngle = Degrees.of(90); // Arbitrary angle (change)
    public static final TalonFXConfiguration kArmTalonFXConfiguration = new TalonFXConfiguration();

    static {
      kArmTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = kArmEncoderID;
      kArmTalonFXConfiguration.Feedback.FeedbackSensorSource =
          FeedbackSensorSourceValue.FusedCANcoder;
      kArmTalonFXConfiguration.Feedback.SensorToMechanismRatio = 1.0;
      kArmTalonFXConfiguration.Feedback.RotorToSensorRatio = 1; // Don't know yet
    }

    public static final CANcoderConfiguration kArmCANCoderConfig = new CANcoderConfiguration();

    static {
      kArmCANCoderConfig.MagnetSensor.SensorDirection =
          SensorDirectionValue.CounterClockwise_Positive;
      kArmCANCoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(0)); // Change offset
    }

    public static final Slot0Configs kArmSlot0Configs = kArmTalonFXConfiguration.Slot0;

    static {
      kArmSlot0Configs.kG = 0; // output to overcome gravity (output)
      kArmSlot0Configs.kS = 0; // output to overcome static friction (output)
      kArmSlot0Configs.kV = 0; // output per unit of target velocity (output/rps)
      kArmSlot0Configs.kA = 0; // output per unit of target acceleration (output/(rps/s))
      kArmSlot0Configs.kP = 0; // output per unit of error in position (output)
      kArmSlot0Configs.kI = 0; // output per unit of integrated error in position (output)
      kArmSlot0Configs.kD = 0; // output per unit of error in velocity (output/rps)
    }

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
    public static final MotionMagicConfigs kMotionMagicConfig =
        kArmTalonFXConfiguration.MotionMagic;

    static {
      kMotionMagicConfig.MotionMagicCruiseVelocity =
          0; // peak velocity of the profile; set to 0 to target the
      // system’s max velocity
      kMotionMagicConfig.MotionMagicExpo_kV =
          0; // voltage required to maintain a given velocity, in V/rps
      kMotionMagicConfig.MotionMagicExpo_kA =
          0; // voltage required to maintain a given velocity, in V/rps
    }

    public static final CurrentLimitsConfigs kCurrentLimitConfig =
        kArmTalonFXConfiguration.CurrentLimits;

    static {
      kCurrentLimitConfig.StatorCurrentLimit = 80; // current limit in amps
      kCurrentLimitConfig.StatorCurrentLimitEnable = true; // enable current limiting
    }

    public static final AngularVelocity UNSAFE_SPEED = RotationsPerSecond.of(1); // 1 rad/s
    public static final Temperature MAX_TEMPERATURE = Celsius.of(90); // Max rated temperature
  }
}
