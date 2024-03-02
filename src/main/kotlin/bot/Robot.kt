package bot


import kotlin.math.abs
import com.revrobotics.CANSparkFlex
import com.revrobotics.SparkPIDController
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
//import system.Console

// CAN ids:
// 1,2,3,4: swerve drive br, bl, fl, fr
// 5,6,7,8: swerve angle br, bl, fl, fr
// 9: elevator
// 10: handoff
// 11, 12: shooter (idr which is which)
// Unset, but planned:
// 13: pivot
// 14: intake pivot
// 15: intake motor
// 16, 17: climbers

class Robot : TimedRobot() {
	private var autoCmd: Command? = null
	private var robotContainer: RobotContainer? = null

	private lateinit var intake: Intake
	private lateinit var swerve: Swerve
	private lateinit var elevator: Elevator
	private lateinit var controller: XboxController
	private lateinit var shooter: Shooter
	private lateinit var motor: CANSparkFlex

	private lateinit var motors: List<CANSparkFlex>
	override fun robotInit() {
		robotContainer = RobotContainer()
		// intake = Intake()
		swerve = Swerve()
		intake = Intake()
 		/*motors = arrayOf(11, 12).zip(arrayOf(true, true)) { id, inverted -> CANSparkFlex(id, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kCoast)
		setSmartCurrentLimit(80)
		enableVoltageCompensation(12.0)
		setInverted(inverted)
	}}*/

		elevator = Elevator()
		controller = XboxController(0)
/*motor = CANSparkFlex(10, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kBrake)
		setSmartCurrentLimit(80)
		enableVoltageCompensation(12.0)
	}*/

	}
	override fun robotPeriodic() {
		/*val pose = swerve.getPose()
		val transError = Translation2d(0.0, 0.0).minus(pose.getTranslation())
		SmartDashboard.putNumber("swerve x pos", pose.getTranslation().getX())
		SmartDashboard.putNumber("swerve y pos", pose.getTranslation().getY())
		if (controller.getRightBumper() && transError.getNorm() > 0.5) {
			val targetAngle = transError.getAngle()
			val error = targetAngle.minus(pose.getRotation()).getRadians()
			var clippedError = error % (2.0 * Math.PI)
			if (clippedError > Math.PI) { clippedError -= 2.0 * Math.PI }
			if (clippedError < -Math.PI) { clippedError += 2.0 * Math.PI }
			val velocity = swerve.getVelocity()
			// trust
			val derivative = (transError.getY() * velocity[0] - transError.getX() * velocity[1]) / (transError.getNorm() * transError.getNorm())
			swerve.applyGoal(Swerve.Goal.Drive(Translation2d(-controller.getLeftY() * Swerve.maxLinVel, -controller.getLeftX() * Swerve.maxLinVel), 3.0 * clippedError + derivative, true))
		} else {
			swerve.applyGoal(Swerve.Goal.Drive(Translation2d(-controller.getLeftY() * Swerve.maxLinVel, -controller.getLeftX() * Swerve.maxLinVel), -controller.getRightX() * Swerve.maxAngVel, true))
		}*/

		/*if (controller.getYButton()) {
			motor.set(0.5)
			motors[0].set(1.0)
			motors[1].set(1.0)
		} else {
			motor.set(0.0)
			motors[0].set(0.0)
			motors[1].set(0.0)
		}*/
		if (controller.getXButton()) {
			elevator.applyGoal(Elevator.Goal.Amp)
		} else if (controller.getAButton()) {
			elevator.applyGoal(Elevator.Goal.Home)
		} else {
			elevator.clearGoal()
		}

		if (true){//controller.getYButton()) {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Handoff, Intake.Rollers.Goal.Coast))
		}
		intake.logPos()
		// Intake controls commented
		/*if (controller.getXButton()) {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Intake))
		} else {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Retracted, Intake.Rollers.Goal.Brake))
		}*/
		if (controller.getLeftBumperPressed()) {
			swerve.setPose(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
		}
		
		CommandScheduler.getInstance().run()
	}

	override fun autonomousInit() {
		autoCmd = robotContainer?.getAutonomousCommand()
		autoCmd?.schedule()
	}

	override fun teleopInit() {
		System.out.println("hello!!!!");
		autoCmd?.cancel() 
	}

	override fun testInit() { CommandScheduler.getInstance().cancelAll() }
}
