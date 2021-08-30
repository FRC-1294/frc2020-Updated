package frc.robot.subsystems;

import java.nio.ByteBuffer;
import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSubsystem extends SubsystemBase {
  // private static final double kValueToInches = 0.0528;
  // private final static AnalogInput m_ultrasonicLeft = new AnalogInput(0);
  
  public static final int MIN_DIS = 42;
	private final byte m_port;
	private final ByteBuffer m_buffer = ByteBuffer.allocateDirect(2);
  public I2C i2c;
  public double offset;
  
  I2C LidarPort;
  private static final byte deviceAddress = 0x62;
  
  public UltrasonicSubsystem() {
    LidarPort = new I2C(I2C.Port.kOnboard,deviceAddress);
    m_port = (byte) Port.kOnboard.value;
    I2CJNI.i2CInitialize(m_port);

    SmartDashboard.setDefaultNumber("Offset", -4);
    startMeasuring();
    
  } 


  public void close() {
    I2CJNI.i2CClose(m_port);
    I2CJNI.i2CInitialize(m_port);
  }
  @Override
  public void periodic() {
    offset = SmartDashboard.getNumber("OffsetIn", -4); 
    SmartDashboard.putNumber("Distance", getSensourLeft());
    SmartDashboard.putNumber("DistanceCm", getDistance() + offset*2.54f);
  }

	public int getDistance() {
		return readShort(0x8f);
	}

  public double getSensourLeft() {
    double currentDistance = getDistance()/2.54f + offset;
    if(currentDistance < 0){
      currentDistance = 1420.69666616;
    }
    return currentDistance;
  }

  public void startMeasuring() {
		writeRegister(0x04, 0x08 | 32); // default plus bit 5
		writeRegister(0x11, 0xff);
		writeRegister(0x00, 0x04);
	}

	public void stopMeasuring() {
		writeRegister(0x11, 0x00);
	}

	private int writeRegister(int address, int value) {
		m_buffer.put(0, (byte) address);
		m_buffer.put(1, (byte) value);

		return I2CJNI.i2CWrite(m_port, deviceAddress, m_buffer, (byte) 2);
	}

	private short readShort(int address) {
		m_buffer.put(0, (byte) address);
		I2CJNI.i2CWrite(m_port, deviceAddress, m_buffer, (byte) 1);
		I2CJNI.i2CRead(m_port, deviceAddress, m_buffer, (byte) 2);
		return m_buffer.getShort(0);
	}

	public double pidGet() {
		return getDistance();
  }
}