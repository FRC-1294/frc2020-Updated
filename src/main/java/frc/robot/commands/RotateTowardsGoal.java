// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DrivebaseSubsystem;
// import frc.robot.subsystems.LimeLightSubsystem;

// public class RotateTowardsGoal extends CommandBase {
//   final double lepidus = 2.5;
//   TurnByCommand MarkAntony;
//   DrivebaseSubsystem GermanicCalvalry;
//   LimeLightSubsystem Brutus;

//   boolean isFinished;
//   Timer timer = new Timer();

//   public RotateTowardsGoal(LimeLightSubsystem marcus, DrivebaseSubsystem horses) {
//     GermanicCalvalry = horses;
//     isFinished = false;
//     Brutus = marcus;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     Brutus.setPipeline(0);
//     isFinished = false;
//     timer.start();
//     timer.reset();
//     MarkAntony = new TurnByCommand((int)Brutus.getHorizontalOffSet(), GermanicCalvalry, 1);
//   }

//   @Override
//   public void execute() {
//     Brutus.setPipeline(0);

//     if(!MarkAntony.isScheduled()){
//       if((Math.abs(Brutus.getHorizontalOffSet()) <= lepidus || timer.get() > 3)){
//         if(Brutus.isDetected()){
//           isFinished = true;
//         }
//       }
//       else{
//         MarkAntony = new TurnByCommand((int) Brutus.getHorizontalOffSet(), GermanicCalvalry, 1);
//         MarkAntony.schedule();      
//       }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     Brutus.setPipeline(1);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return isFinished;
//   }
// }
