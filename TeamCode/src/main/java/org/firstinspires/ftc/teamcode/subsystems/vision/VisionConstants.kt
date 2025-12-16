package org.firstinspires.ftc.teamcode.subsystems.vision

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

object VisionConstants {

    val CAMERA_POSITION: Position = Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0)

    val CAMERA_ANGLES: YawPitchRollAngles = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0)

}