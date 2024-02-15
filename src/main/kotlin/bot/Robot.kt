package bot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

class Robot : TimedRobot() {
	private var autoCmd: Command? = null
	private var robotContainer: RobotContainer? = null

	private lateinit var intake: Intake
	private lateinit var swerve: Swerve
	private lateinit var controller: XboxController

	override fun robotInit() {
		robotContainer = RobotContainer()
		// intake = Intake()
		swerve = Swerve()
		controller = XboxController(0)
	}
	override fun robotPeriodic() {
		controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.5)
		swerve.applyGoal(Swerve.Goal.Drive(Translation2d(-controller.getLeftY(), -controller.getLeftX()), -controller.getRightX(), true))
		// Intake controls commented
		/*if (controller.getXButton()) {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Intake))
		} else {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Retracted, Intake.Rollers.Goal.Brake))
		}*/
		if (controller.getLeftBumperPressed()) {
			swerve.resetAbsEncoders()
		}
		
		CommandScheduler.getInstance().run()
	}

	override fun autonomousInit() {
		autoCmd = robotContainer?.getAutonomousCommand()
		autoCmd?.schedule()
	}

	override fun teleopInit() { autoCmd?.cancel() }

	override fun testInit() { CommandScheduler.getInstance().cancelAll() }
}
