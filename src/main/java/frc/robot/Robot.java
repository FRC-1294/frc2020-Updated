package frc.robot;

import frc.robot.commands.DictatorLocator;
import frc.robot.commands.MoveByCommand;
import frc.robot.subsystems.DistanceSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.GameMechSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Command m_autonomousCommand;
  public static DistanceSubsystem ultrasonic;
  public static LimeLightSubsystem limelight;
  public static GameMechSubsystem gameMech;
  public static DrivebaseSubsystem driveBase;
  public static MoveByCommand chacharealmooth;

  public static boolean inAuto = false;
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    gameMech = new GameMechSubsystem();
    driveBase = new DrivebaseSubsystem();
    ultrasonic = new DistanceSubsystem();
    limelight = new LimeLightSubsystem();
    
    driveBase.setFrontLeftSpeed(0);
    driveBase.setFrontRightSpeed(0);
    driveBase.setRearLeftSpeed(0); 
    driveBase.setRearRightSpeed(0);
    gameMech.setZero();
    limelight.setPipeline(1);
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  
  @Override
  public void disabledInit() {
    // CommandScheduler.getInstance().cancelAll();
    driveBase.setFrontLeftSpeed(0);
    driveBase.setFrontRightSpeed(0);
    driveBase.setRearLeftSpeed(0);
    driveBase.setRearRightSpeed(0);
    gameMech.setZero();
    limelight.setPipeline(1);
  }

  @Override
  public void disabledPeriodic() {
    driveBase.setFrontLeftSpeed(0);
    driveBase.setFrontRightSpeed(0);
    driveBase.setRearLeftSpeed(0); 
    driveBase.setRearRightSpeed(0);
    gameMech.setZero();
    limelight.setPipeline(1);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = new DictatorLocator(limelight, driveBase);//new TurnByCommand(360, m_driveAuto, 0);//new AlignToShoot(m_driveAuto, ultrasonic, letsShoot, cassius, 112, true);//new AutoNavCommand(m_driveAuto, ultrasonic, letsShoot, cassius);//new AutoNavCommand(m_driveAuto, ultrasonic, letsShoot, cassius);
    

    // schedule the autonomous command (example)
    if (!m_autonomousCommand.isScheduled()) {
      //m_autonomousCommand = new DictatorLocator(cassius, m_driveAuto);
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomo us.
   */
  @Override
  public void autonomousPeriodic() {
    inAuto = true;
    // if (!m_autonomousCommand.isFinished() && !m_autonomousCommand.isScheduled()) {
    //   m_autonomousCommand =  new AutoNavCommand(driveAuto, ultrasonic);
    //   m_autonomousCommand.schedule();
    // }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    ultrasonic.close();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    inAuto = false;
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
