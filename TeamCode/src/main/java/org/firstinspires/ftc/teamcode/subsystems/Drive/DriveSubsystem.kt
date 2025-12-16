package org.firstinspires.ftc.teamcode.subsystems.Drive

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.drivebase.MecanumDrive
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.hardware.RevIMU
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveKinematics
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveOdometry
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
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.BACK_LEFT_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.BACK_RIGHT_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Distance.BACK_LEFT_DISTANCE_FROM_CENTER
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Distance.BACK_RIGHT_DISTANCE_FROM_CENTER
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Distance.FRONT_LEFT_DISTANCE_FROM_CENTER
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.Distance.FRONT_RIGHT_DISTANCE_FROM_CENTER
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.FRONT_LEFT_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants.FRONT_RIGHT_MOTOR_TYPE
import kotlin.math.PI

class DriveSubsystem(var hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {

    init {
        name = "drive subsystem"
    }

    val frontLeftMotor = HaDcMotor(
        hardwareMap,
        FRONT_LEFT_MOTOR_ID,
        FRONT_LEFT_MOTOR_TYPE
    )

    val frontRightMotor = HaDcMotor(
        hardwareMap,
        FRONT_RIGHT_MOTOR_ID,
        FRONT_RIGHT_MOTOR_TYPE
    )

    val backLeftMotor = HaDcMotor(
        hardwareMap,
        BACK_LEFT_MOTOR_ID,
        BACK_LEFT_MOTOR_TYPE
    )

    val backRightMotor = HaDcMotor(
        hardwareMap,
        BACK_RIGHT_MOTOR_ID,
        BACK_RIGHT_MOTOR_TYPE
    )

    val gyro = RevIMU(hardwareMap).apply {
        revIMU.angularVelocity.angleUnit = UnnormalizedAngleUnit.RADIANS
        revIMU.velocity.unit = DistanceUnit.METER
    }


    val drive = MecanumDrive(
        frontLeftMotor,
        frontRightMotor,
        backLeftMotor,
        backRightMotor
    )

    val kinematics = MecanumDriveKinematics(
        FRONT_LEFT_DISTANCE_FROM_CENTER,
        FRONT_RIGHT_DISTANCE_FROM_CENTER,
        BACK_LEFT_DISTANCE_FROM_CENTER,
        BACK_RIGHT_DISTANCE_FROM_CENTER
    )

    var poseEstimator = MecanumDriveOdometry(
        kinematics,
        gyro.heading.degrees
    )


    // --- Robot state getters ---

    val robotHeading: Rotation2d get() = gyro.rotation2d

    val robotPitch: Rotation2d get() = gyro.angles[0].degrees

    val robotVelocity: ChassisSpeeds
        get() = ChassisSpeeds(
            gyro.revIMU.velocity.xVeloc,
            gyro.revIMU.velocity.yVeloc,
            gyro.revIMU.angularVelocity.zRotationRate.toDouble()
        )

    val robotPose: Pose2d get() = poseEstimator.poseMeters

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
}