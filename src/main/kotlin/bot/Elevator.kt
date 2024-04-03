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
		//setPosition(0.0)
	}

	private val controller = motor.getPIDController().apply {
		// TODO: Tune
		// Position PID
		// gives 15% output with 1cm error
		setP(10.0) // 15
		setI(0.0)
		setD(0.1)
		setOutputRange(-1.0, 1.0)
	}

	sealed interface Goal {
		// around 0.86-0.873 is absolute max extension
		val pos: Double

		object Home : Goal { override val pos = -0.24 }
		object High : Goal { override val pos = 0.0 }
		object Handoff : Goal { override val pos = 0.825-0.235-0.0635 }//0.15-0.15 }
		object Amp : Goal { override val pos = 0.40 }
		object Trap : Goal { override val pos = 0.825-0.0635 }
		object Defense : Goal { override val pos = 0.75-0.0635 }
		data class Other(override val pos: Double) : Goal
	}

	data class State(val pos: Double, val atGoal: Boolean)

	private val profile = TrapezoidProfile(TrapezoidProfile.Constraints(1.3, 1.0)) // 2.7
	private var lastGoal: Goal? = null
	private var timer = Timer()
	private lateinit var initState: TrapezoidProfile.State

	override fun applyGoal(goal: Goal): State {
		if (goal != lastGoal) {
			timer.restart()
			initState = TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity())
		}
		lastGoal = goal
		val setpoint = profile.calculate(timer.get(), initState, TrapezoidProfile.State(goal.pos, 0.0))
		controller.setReference(setpoint.position, ControlType.kPosition, 0, (setpoint.velocity / freeSpeed), SparkPIDController.ArbFFUnits.kPercentOut)
		SmartDashboard.putNumber("elevator position", encoder.getPosition())
		SmartDashboard.putNumber("elevator setpoint position", setpoint.position)
		SmartDashboard.putNumber("elevator target position", goal.pos)
		return State(
			encoder.getPosition(),
			abs(goal.pos - encoder.getPosition()) < 0.0 && abs(encoder.getVelocity()) < 0.0)
	}

	// temp for testing
	fun clearGoal() {
		lastGoal = null
	}

	override fun disable() {
		lastGoal = null
	}
}
