package bot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.abs
import com.revrobotics.CANSparkFlex
import com.revrobotics.SparkPIDController
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer

class Elevator2: StateSystem<Elevator2.Goal, Elevator2.State> {
	companion object {
		// Number of belt teeth moved per motor revolution
		// 9:1 reduction, followed by a 25 tooth pulley
		const val gearing = 25.0 / 9.0
		// 0.005 because HTD-5M belt has a tooth pitch of 5mm/0.005m
		const val posFactor = gearing * 0.005 // motor rotations -> elevator meters
		const val velFactor = posFactor / 60.0 // motor rpm -> elevator m/s
		const val freeSpeed = 6784.0 * velFactor
	}

	private val motor = CANSparkFlex(9, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kBrake)
		setSmartCurrentLimit(60)
		enableVoltageCompensation(12.0)
		setInverted(true)
	}

	private val encoder = motor.getEncoder().apply {
		setPositionConversionFactor(posFactor)
		setVelocityConversionFactor(velFactor)
	}

	private val controller = motor.getPIDController().apply {
		// TODO: Tune
		// Position PID
		// gives 15% output with 3cm error
		setP(10.0)
		setI(0.0)
		setD(0.0)
		setOutputRange(-1.0, 1.0)
	}

	sealed interface Goal {
		val pos: Double

		object Home : Goal { override val pos = 0.05 }
		//object Handoff : Goal { override val pos = 0.0 }
		object Amp : Goal { override val pos = 0.5 }
		//object Trap : Goal { override val pos = 0.0 }
		//object Defense : Goal { override val pos = 0.0 }
		//data class Other(override val pos: Double) : Goal
	}

	data class State(val pos: Double, val atGoal: Boolean)

	private val profile = TrapezoidProfile(TrapezoidProfile.Constraints(1.0, 1.0))

	override fun applyGoal(goal: Goal): State {
		//SmartDashboard.putNumber("timer", timer.get())
		// OLD: Expected initial output: P * 0.0002 + (0.02 / freespeed)
		val setpoint = profile.calculate(0.02, TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity()), TrapezoidProfile.State(goal.pos, 0.0))
		// TODO: gravity aFF
		controller.setReference(setpoint.position, ControlType.kPosition, 0, (setpoint.velocity / freeSpeed) + Math.copySign(0.1, setpoint.velocity) + 0.05, SparkPIDController.ArbFFUnits.kPercentOut)
		SmartDashboard.putNumber("sp vel", setpoint.velocity)
		SmartDashboard.putNumber("elev pos", encoder.getPosition())
		SmartDashboard.putNumber("sp pos", setpoint.position)
		SmartDashboard.putNumber("elev pos 2", encoder.getPosition())
		return State(
			encoder.getPosition(),
			abs(goal.pos - encoder.getPosition()) < 0.0 && abs(encoder.getVelocity()) < 0.0)
	}

	// temp for testing
	fun clearGoal() {
	}

	override fun disable() {
	}
}
