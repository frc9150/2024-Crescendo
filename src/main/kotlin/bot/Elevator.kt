package bot

import kotlin.math.abs
import com.revrobotics.CANSparkFlex
import com.revrobotics.SparkPIDController
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.math.trajectory.TrapezoidProfile

class Elevator: StateSystem<Elevator.Goal, Elevator.State> {
	companion object {
		// Number of belt teeth moved per motor revolution
		// 9:1 reduction, followed by a 25 tooth pulley
		const val gearing = 25.0 / 9.0
		// 0.005 because HTD-5M belt has a tooth pitch of 5mm/0.005m
		const val posFactor = gearing * 0.005 // motor rotations -> elevator meters
		const val velFactor = posFactor / 60.0 // motor rpm -> elevator m/s
		const val freeSpeed = 6784.0 * velFactor
	}

	private val motor = CANSparkFlex(-1, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kBrake)
		setSmartCurrentLimit(25)
		enableVoltageCompensation(12.0)
	}

	private val encoder = motor.getEncoder().apply {
		setPositionConversionFactor(posFactor)
		setVelocityConversionFactor(velFactor)
	}

	private val controller = motor.getPIDController().apply {
		// TODO: Tune
		// Position PID
		setP(0.1)
		setI(0.0)
		setD(0.0)
		setOutputRange(-1.0, 1.0)
	}

	sealed interface Goal {
		val pos: Double

		object Home : Goal { override val pos = 0.0 }
		object Handoff : Goal { override val pos = 0.0 }
		object Amp : Goal { override val pos = 0.0 }
		object Trap : Goal { override val pos = 0.0 }
		object Defense : Goal { override val pos = 0.0 }
		data class Other(override val pos: Double) : Goal
	}

	data class State(val pos: Double, val atGoal: Boolean)

	private val profile = TrapezoidProfile(TrapezoidProfile.Constraints(1.0, 1.0))

	override fun applyGoal(goal: Goal): State {
		val setpoint = profile.calculate(0.02, TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity()), TrapezoidProfile.State(goal.pos, 0.0))
		// TODO: Velocity feedforward, gravity aFF
		controller.setReference(setpoint.position, ControlType.kPosition, 0, (setpoint.velocity / freeSpeed) * 12.0, SparkPIDController.ArbFFUnits.kVoltage)
		return State(
			encoder.getPosition(),
			abs(goal.pos - encoder.getPosition()) < 0.0 && abs(encoder.getVelocity()) < 0.0)
	}
}
