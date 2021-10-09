package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
  public static GameMechSubsystem gameMech;
  public static DrivebaseSubsystem driveBase;

  public static boolean inAuto = false;
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    gameMech = new GameMechSubsystem();
    driveBase = new DrivebaseSubsystem();
    
    driveBase.setFrontLeftSpeed(0);
    driveBase.setFrontRightSpeed(0);
    driveBase.setRearLeftSpeed(0); 
    driveBase.setRearRightSpeed(0);
    gameMech.setZero();
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  
  @Override
  public void disabledInit() {
    // CommandScheduler.getInstance().cancelAll();
    driveBase.stop();
    gameMech.setZero();
  }

  @Override
  public void disabledPeriodic() {
    driveBase.stop();
    gameMech.setZero();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {}

  /**
   * This function is called periodically during autonomo us.
   */
  @Override
  public void autonomousPeriodic() {
    inAuto = true;
  }

  @Override
  public void teleopInit() {}

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
