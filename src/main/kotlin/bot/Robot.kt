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
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
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
	private lateinit var controller2: XboxController
	private lateinit var shooter: Shooter
	private lateinit var motor: CANSparkFlex
	private lateinit var pivot: Pivot

	private lateinit var motors: List<CANSparkFlex>
	private lateinit var climbs: List<CANSparkFlex>
	private lateinit var motori: CANSparkFlex
	override fun robotInit() {
		robotContainer = RobotContainer()
		// intake = Intake()
		swerve = Swerve()
		//intake = Intake()
		pivot = Pivot()
 		motors = arrayOf(11, 12).zip(arrayOf(true, true)) { id, inverted -> CANSparkFlex(id, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kCoast)
		setSmartCurrentLimit(80)
		enableVoltageCompensation(12.0)
		setInverted(inverted)
	}}
	 	climbs = arrayOf(16, 17).zip(arrayOf(true, false)) { id, inverted -> CANSparkFlex(id, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kCoast)
		setSmartCurrentLimit(80)
		enableVoltageCompensation(12.0)
		setInverted(inverted)
	}}

		elevator = Elevator()
		controller = XboxController(0)
		controller2 = XboxController(1)
motor = CANSparkFlex(10, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kBrake)
		setSmartCurrentLimit(80)
		enableVoltageCompensation(12.0)
	}
	motori = CANSparkFlex(15, MotorType.kBrushless).apply {
			restoreFactoryDefaults()
			setIdleMode(IdleMode.kBrake)
			setSmartCurrentLimit(40)
			enableVoltageCompensation(12.0)
		}

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
		} else {*/
		//}

		//if (controller.getBButton()) {
	//		pivot.applyGoal(Pivot.Goal.Amp)
	//	}
		pivot.dumpPos()
		/*if (controller2.getYButton()) {
			climbs[0].set(1.0)
			climbs[1].set(1.0)
		} else if (controller2.getBButton()) {
			climbs[0].set(-1.0)
			climbs[1].set(-1.0)
		}else{
			climbs[0].set(0.0)
			climbs[1].set(0.0)
		}*/

		
		
		CommandScheduler.getInstance().run()
	}
	private var autoStart = 0.0
	override fun autonomousInit() {
		autoCmd = robotContainer?.getAutonomousCommand()
		autoCmd?.schedule()
		autoStart = Timer.getFPGATimestamp()
	}
	override fun autonomousPeriodic() {
		val time = Timer.getFPGATimestamp() - autoStart
		if (time < 4.0) {
			elevator.applyGoal(Elevator.Goal.Handoff)
		} else {
			elevator.applyGoal(Elevator.Goal.Home)
		}
		if (time > 1.5 && time < 4.0) {
			motors[0].set(1.0)
			motors[1].set(1.0)
		} else {
			motors[0].set(0.0)
			motors[1].set(0.0)
		}
		if (time > 2.5 && time < 4.0) {
			motor.set(0.3)
		} else {
			motor.set(0.0)
		}
		if (time > 5.0) {
			swerve.applyGoal(Swerve.Goal.Drive(Translation2d(-0.4, 0.0), 0.0, true))
		}
	}

	override fun teleopInit() {
		System.out.println("hello!!!!");
		autoCmd?.cancel()
		swerve.setRotation(swerve.getPose().getRotation().plus(Rotation2d.fromDegrees(180.0)))
	}

	override fun teleopPeriodic() {
		val multiplier: Double = if (controller.getLeftBumper()) { 0.5 } else { 1.0 }
		if (controller.getRightBumper()) {
			var targetAngle = Rotation2d.fromDegrees(-60.0)
			val ally = DriverStation.getAlliance()
			if (ally.isPresent() && ally.get() == Alliance.Red) {
				targetAngle = Rotation2d.fromDegrees(60.0)
			}

			val error = targetAngle.minus(swerve.getPose().getRotation()).getRadians()
			var clippedError = error % (2.0 * Math.PI)
			if (clippedError > Math.PI) { clippedError -= 2.0 * Math.PI }
			if (clippedError < -Math.PI) { clippedError += 2.0 * Math.PI }
			swerve.applyGoal(Swerve.Goal.Drive(Translation2d(multiplier * -controller.getLeftY() * Swerve.maxLinVel, multiplier * -controller.getLeftX() * Swerve.maxLinVel), 3.5 * clippedError, true))
		} else {
			swerve.applyGoal(Swerve.Goal.Drive(Translation2d(multiplier * -controller.getLeftY() * Swerve.maxLinVel, multiplier * -controller.getLeftX() * Swerve.maxLinVel), multiplier * -controller.getRightX() * Swerve.maxAngVel, true))
		}
		if (controller2.getRightBumper()) {
			//motor.set(-0.5)
			motors[0].set(-0.5)
			motors[1].set(-0.5)
		} else if (false){//controller.getLeftBumper()) {
			motor.set(-0.3)
			//motori.set(1.0)
			motors[0].set(
				0.0)
			motors[1].set(0.0)
		} else if (controller2.getBButton()) {
			motor.set(-0.2)
			//motori.set(1.0)
			motors[0].set(
				0.0)
			motors[1].set(0.0)
		} else if (false){//controller2.getLeftBumper()) {
			motor.set(0.3)
			//motors[0].set(0.0)
			//motors[1].set(0.0)
			motori.set(1.0)	
		} else {
			motor.set(0.0)
			motors[0].set(0.0)
			motors[1].set(0.0)
			motori.set(0.0)		
		}
		if (controller2.getLeftBumper()) {
			motors[0].set(1.0)
			motors[1].set(1.0)
		} else if (controller2.getStartButton()) {
			motors[0].set(0.375)
			motors[1].set(0.1875)
		}
		if(controller2.getYButton()) {
			motor.set(0.3)
		}
		if (controller2.getXButton()) {
			elevator.applyGoal(Elevator.Goal.Handoff)
		} else if (controller2.getAButton()) {
			elevator.applyGoal(Elevator.Goal.Amp)
		} else {
			elevator.applyGoal(Elevator.Goal.Home)
			/*elevator.clearGoal()*/
		}

		/*if (controller.getYButton()) {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Handoff, Intake.Rollers.Goal.Coast))
		}*/
		//intake.logPos()
		// Intake controls commented
		/*if (controller.getXButton()) {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Out, Intake.Rollers.Goal.Intake))
		} else {
			intake.applyGoal(Intake.Goal(Intake.Pivot.Goal.Retracted, Intake.Rollers.Goal.Brake))
		}*/
		if (controller.getStartButtonPressed()) {
			swerve.setPose(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
		}
	}

	override fun testInit() { CommandScheduler.getInstance().cancelAll() }
}
