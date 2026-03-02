package frc.robot.util;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CANSparkFlexSendable extends SparkFlex implements Sendable {
  private CANSparkFlexSendableAdapter m_sendableAdapter;

  public CANSparkFlexSendable(int deviceId, MotorType type) {
    super(deviceId, type);
    m_sendableAdapter = new CANSparkFlexSendableAdapter(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    m_sendableAdapter.initSendable(builder);
  }  
}
