package bot

//import edu.wpi.first.wpilibj2.command.Subsystem

import kotlin.math.abs
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType
//import edu.wpi.first.wpilibj.Timer
//import edu.wpi.first.math.trajectory.TrapezoidProfile

class Intake : StateSystem<Intake.Goal, Intake.State> {
	data class Goal(val pivot: Pivot.Goal, val rollers: Rollers.Goal)

	data class State(val pivot: Pivot.State, val rollers: Rollers.State)

	private val pivot = Pivot()
	private val rollers = Rollers()

	override fun applyGoal(goal: Goal) = State(pivot.applyGoal(goal.pivot), rollers.applyGoal(goal.rollers))

	override fun disable() {
		pivot.disable()
		rollers.disable()
	}

	// TODO: incorporate pivot gearing: pivot is geared 4:1, and the ratio from the pulleys is 18t:18t
	class Pivot : StateSystem<Pivot.Goal, Pivot.State> {
		companion object {
			// 16:1 gear ratio
			const val gearing = 0.0625;
			const val posFactor = gearing * (2 * Math.PI) // motor rotations -> pivot radians
			const val velFactor = posFactor / 60.0 // motor rpm -> pivot rad/s
		}

		sealed interface Goal {
			val angle: Double

			object Out : Goal { override val angle = 0.0 }
			object Retracted : Goal { override val angle = 0.0 }
			object Handoff : Goal { override val angle = 0.0 }
			data class Other(override val angle: Double) : Goal
		}

		data class State(val angle: Double, val atGoal: Boolean)

		private val motor = CANSparkFlex(-1, MotorType.kBrushless).apply {
			restoreFactoryDefaults()
			setIdleMode(IdleMode.kBrake)
			setSmartCurrentLimit(15)
			enableVoltageCompensation(11.0)
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

		override fun applyGoal(goal: Goal): State {
			controller.setReference(goal.angle, ControlType.kPosition)
			return State(encoder.getPosition(), abs(goal.angle - encoder.getPosition()) < 0.0 && abs(encoder.getVelocity()) < 0.0)
		}
	}

	class Rollers : StateSystem<Rollers.Goal, Rollers.State> {
		sealed interface Goal {
			object Intake : Goal
			object Eject : Goal
			object Brake : Goal
			object Coast : Goal
			data class Other(val power: Double) : Goal
		}
		
		data class State(val vel: Double)

		private val motor = CANSparkFlex(-1, MotorType.kBrushless).apply {
			restoreFactoryDefaults()
			setIdleMode(IdleMode.kBrake)
			setSmartCurrentLimit(15)
			enableVoltageCompensation(11.0)
		}

		private val encoder = motor.getEncoder()

		private val controller = motor.getPIDController().apply {
			// TODO: Tune
			// Position PID
			setP(0.1)
			setI(0.0)
			setD(0.0)
			setOutputRange(-1.0, 1.0)
		}

		private var lastGoal: Goal = Goal.Coast
		private var holdPos = 0.0

		override fun applyGoal(goal: Goal): State {
			when (goal) {
				is Goal.Intake -> {
					motor.set(1.0)
				}
				is Goal.Eject -> {
					motor.set(-1.0)
				}
				is Goal.Brake -> {
					if (!(lastGoal is Goal.Brake)) {
						holdPos = encoder.getPosition()
					}
					controller.setReference(holdPos, ControlType.kPosition)
				}
				is Goal.Coast -> {
					motor.set(0.0)
				}
				is Goal.Other -> {
					motor.set(goal.power)
				}
			}
			lastGoal = goal
			return State(encoder.getVelocity())
		}

		override fun disable() {
			// Do this so that if commanded to brake after re-enabled, the target position will be reset
			lastGoal = Goal.Coast
		}
	}
}