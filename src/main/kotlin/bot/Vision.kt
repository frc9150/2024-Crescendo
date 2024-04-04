package bot

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer

class Vision : StateSystem<Vision.Goal, Vision.State> {
	val field = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)
	val robotToCam = Transform3d(
		Translation3d(Units.inchesToMeters(5.0), Units.inchesToMeters(13.25), Units.inchesToMeters(21.25 + 4.75)),
		Rotation3d(0.0, Units.degreesToRadians(-14.75), 0.0))
	val camToRobot = robotToCam.inverse()

	val limelight = LimelightHelpers.getLimelightNTTable("")
	// tx (degrees), right+
	val tx = limelight.getEntry("tx")
	// ty (degrees), up+
	val ty = limelight.getEntry("ty")
	// tag id
	val tid = limelight.getEntry("tid")
	// pipeline latency (ms)
	val tl = limelight.getEntry("tl")
	// capture latency (ms)
	val cl = limelight.getEntry("cl")

	data class Goal(val rotationBuffer: TimeInterpolatableBuffer<Rotation2d>)
	data class State(val pose: Pose2d?, val timestamp: Double)

	private var lastTimestamp = 0.0

	override fun applyGoal(goal: Goal): State {
		val pipeLatency = tl.getDouble(0.0)
		val captureLatency = cl.getDouble(0.0)
		val lastChange = tx.getLastChange()
		val txVal = tx.getDouble(0.0)
		val tyVal = ty.getDouble(0.0)
		val tidVal = tid.getInteger(-1).toInt()
		val timestamp = (lastChange / 1.0e6) - ((pipeLatency + captureLatency) / 1.0e3)
		val rotationOpt = goal.rotationBuffer.getSample(timestamp)
		val tagPoseOpt = field.getTagPose(tidVal)
		// 0.0 signals no vision / bad data
		if (pipeLatency != 0.0 && captureLatency != 0.0 && txVal != 0.0 && tyVal != 0.0 &&
			tidVal >= 1 && tidVal <= 16 && rotationOpt.isPresent() && tagPoseOpt.isPresent()) {
			val rotation = rotationOpt.get()
			val tagPose = tagPoseOpt.get()
			val dZ = tagPose.getZ() - robotToCam.getZ()
			val vAngle = Units.degreesToRadians(tyVal + 14.75)
			// now we have a triangle with opposite leg dZ (height difference) and angle vAngle.
			// dD is the distance from the tag
			val dD = dZ / Math.tan(vAngle)
			// angle from the bot to the tag
			val hAngle = rotation.plus(Rotation2d.fromDegrees(-txVal))
			// x and y from bot to tag
			val dX = hAngle.getCos() * dD
			val dY = hAngle.getSin() * dD

			val botX = tagPose.getX() - dX
			val botY = tagPose.getY() - dY
			return State(Pose2d(Translation2d(botX, botY).plus(camToRobot.getTranslation().toTranslation2d().rotateBy(rotation)), rotation), timestamp)
		}
		return State(null, timestamp)
	}
}
