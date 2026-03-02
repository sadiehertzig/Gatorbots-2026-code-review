package frc.robot.util;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class CANSparkFlexSendableAdapter implements Sendable  {
  private SparkFlex m_motor;

  public CANSparkFlexSendableAdapter(SparkFlex motor) {
    m_motor = motor;
    SendableRegistry.addLW(this, "SparkFlex", motor.getDeviceId());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Motor Controller");
    builder.setActuator(true);
    builder.setSafeState(m_motor::stopMotor);
    builder.addDoubleProperty("Value", m_motor::get, m_motor::set);
  }
}
