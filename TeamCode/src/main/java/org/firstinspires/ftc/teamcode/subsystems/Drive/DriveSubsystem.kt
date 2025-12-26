package org.firstinspires.ftc.teamcode.subsystems.Drive

import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.hardware.RevIMU
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveKinematics
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.alonlib.math.mapRange
import org.firstinspires.ftc.teamcode.alonlib.motors.HaDcMotor
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.radians
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.DRIVE_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Distance.BACK_LEFT_DISTANCE_FROM_CENTER
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Distance.BACK_RIGHT_DISTANCE_FROM_CENTER
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Distance.FRONT_LEFT_DISTANCE_FROM_CENTER
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Distance.FRONT_RIGHT_DISTANCE_FROM_CENTER
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Roadrunner.K_A
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Roadrunner.K_STATIC
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Roadrunner.K_V
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Roadrunner.MOTOR_VELOCITY_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Roadrunner.ROBOT_HEADING_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Roadrunner.ROBOT_MOVEMENT_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Roadrunner.ROBOT_PATH_FOLLOWER_ANGLE_TOLERANCE
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Roadrunner.ROBOT_PATH_FOLLOWER_DISTANCE_TOLERANCE
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner
import kotlin.math.PI

class DriveSubsystem(var hardwareMap: HardwareMap, var telemetry: Telemetry) : MecanumDrive(K_V, K_A, K_STATIC, TRACK_WIDTH.asMeters) {

    val frontLeftMotor = HaDcMotor(
        hardwareMap,
        FRONT_LEFT_MOTOR_ID,
        DRIVE_MOTOR_TYPE
    ).apply {
        setRunMode(Motor.RunMode.VelocityControl)
        setPIDGains(MOTOR_VELOCITY_PID_GAINS)
    }

    val frontRightMotor = HaDcMotor(
        hardwareMap,
        FRONT_RIGHT_MOTOR_ID,
        DRIVE_MOTOR_TYPE
    ).apply {
        setRunMode(Motor.RunMode.VelocityControl)
        setPIDGains(MOTOR_VELOCITY_PID_GAINS)
        runningDirection = Motor.Direction.REVERSE
    }

    val backLeftMotor = HaDcMotor(
        hardwareMap,
        BACK_LEFT_MOTOR_ID,
        DRIVE_MOTOR_TYPE
    ).apply {
        setRunMode(Motor.RunMode.VelocityControl)
        setPIDGains(MOTOR_VELOCITY_PID_GAINS)
    }

    val backRightMotor = HaDcMotor(
        hardwareMap,
        BACK_RIGHT_MOTOR_ID,
        DRIVE_MOTOR_TYPE
    ).apply {
        setRunMode(Motor.RunMode.VelocityControl)
        setPIDGains(MOTOR_VELOCITY_PID_GAINS)
        runningDirection = Motor.Direction.REVERSE

    }

    val batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

    val gyro = RevIMU(hardwareMap).apply {
        revIMU.angularVelocity.angleUnit = UnnormalizedAngleUnit.RADIANS
        revIMU.velocity.unit = DistanceUnit.METER
    }

    var poseEstimator = MecanumLocalizer(this, true)

    val drive = com.seattlesolvers.solverslib.drivebase.MecanumDrive(
        frontLeftMotor,
        frontRightMotor,
        backLeftMotor,
        backRightMotor
    ).apply { localizer = poseEstimator }

    val kinematics = MecanumDriveKinematics(
        FRONT_LEFT_DISTANCE_FROM_CENTER,
        FRONT_RIGHT_DISTANCE_FROM_CENTER,
        BACK_LEFT_DISTANCE_FROM_CENTER,
        BACK_RIGHT_DISTANCE_FROM_CENTER
    )


    // --- Roadrunner ---
    val follower = HolonomicPIDVAFollower(
        ROBOT_MOVEMENT_PID_GAINS,
        ROBOT_MOVEMENT_PID_GAINS,
        ROBOT_HEADING_PID_GAINS,
        Pose2d(ROBOT_PATH_FOLLOWER_DISTANCE_TOLERANCE, ROBOT_PATH_FOLLOWER_ANGLE_TOLERANCE.radians)
    )

    val lastTrackingEncoderPositions = listOf<Int>()

    val lastTrackingEncoderVelocities = listOf<Int>()

    val lastDriveEncoderPositions = listOf<Int>()

    val lastDriveEncoderVelocities = listOf<Int>()


    val pathSequenceRunner = TrajectorySequenceRunner(
        follower,
        ROBOT_MOVEMENT_PID_GAINS,
        batteryVoltageSensor,
        lastDriveEncoderPositions,
        lastDriveEncoderVelocities,
        lastTrackingEncoderPositions,
        lastTrackingEncoderVelocities
    )


    // --- Robot state getters ---

    val robotHeading: Rotation2d get() = poseEstimator.poseEstimate.heading.radians //TODO: check units

    val robotPitch: Rotation2d get() = gyro.angles[0].degrees

    val robotVelocity: Pose2d get() = Pose2d(poseEstimator.poseVelocity!!.x, poseEstimator.poseVelocity!!.y, poseEstimator.poseVelocity!!.heading)

    val robotPose: Pose2d get() = Pose2d(poseEstimator.poseEstimate.x, poseEstimator.poseEstimate.y, poseEstimator.poseEstimate.heading)

    override val rawExternalHeading = gyro.absoluteHeading

    val isInVisionRange get() = false //TODO: implement vision

    // --- Drive Control ---

    fun drive(
        translation: Translation2d,
        turnMultiplier: Double,
        isFieldRelative: Boolean = true,
        useClosedLoopDrive: Boolean = false
    ) {

        if (isFieldRelative) {
            drive.driveFieldCentric(
                translation.x,
                translation.y,
                turnMultiplier,
                gyro.revIMU.angularOrientation.thirdAngle.toDouble()

            )
        } else {
            drive.driveRobotCentric(
                translation.x,
                translation.y,
                turnMultiplier,
                false
            )
        }
    }

    fun stop() {
        drive.stop()
    }

    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds, isFieldRelative: Boolean) {
        if (isFieldRelative) {
            drive.driveFieldCentric(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                mapRange(chassisSpeeds.omegaRadiansPerSecond, 0.0, 2.0 * PI, 0.0, 1.0),
                gyro.revIMU.angularOrientation.thirdAngle.toDouble()
            )
        } else {
            drive.driveRobotCentric(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                mapRange(chassisSpeeds.omegaRadiansPerSecond, 0.0, 2.0 * PI, 0.0, 1.0)
            )
        }
    }

    override fun getWheelPositions(): List<Double> {
        return listOf(frontLeftMotor.distance, frontRightMotor.distance, backLeftMotor.distance, backRightMotor.distance)
    }

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        frontLeftMotor.set(frontLeft)
        frontRightMotor.set(frontRight)
        backLeftMotor.set(rearRight)
        backRightMotor.set(rearLeft)
    }
}