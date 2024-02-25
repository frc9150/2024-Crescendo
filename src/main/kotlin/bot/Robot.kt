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
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
//import system.Console

// CAN ids:
// 1,2,3,4: swerve drive fl, fr, br, bl
// 5,6,7,8: swerve angle fl, fr, br, bl
// 9: elevator
// 10: handoff
// 11, 12: shooter (idr which is which)
// Unset, but planned:
// 13: pivot
// 14, 15: climbers

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
		controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.5)
		swerve.applyGoal(Swerve.Goal.Drive(Translation2d(-controller.getLeftY() * Swerve.maxLinVel, -controller.getLeftX() * Swerve.maxLinVel), -controller.getRightX() * Swerve.maxAngVel, true))

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
		// Intake controls commented
		/*if (controller.getXButton()) {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Intake))
		} else {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Retracted, Intake.Rollers.Goal.Brake))
		}*/
		if (controller.getLeftBumperPressed()) {
			swerve.setRotation(Rotation2d(0.0))
		}
		// -0.567581
		// -0.03302
		
		CommandScheduler.getInstance().run()
	}

	override fun autonomousInit() {
		autoCmd = robotContainer?.getAutonomousCommand()
		autoCmd?.schedule()
	}

	override fun teleopInit() {
		System.out.println("hello!!!!");
		//swerve.resetAbsEncoders()
		autoCmd?.cancel() 
	}

	override fun testInit() { CommandScheduler.getInstance().cancelAll() }
}
