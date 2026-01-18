package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.LIMELIGHT_IP_ADDRESS
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.LIMELIGHT_NAME
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.LIMELIGHT_SERIAL_NUMBER


class Limelight() : Limelight3A(LIMELIGHT_SERIAL_NUMBER, LIMELIGHT_NAME, LIMELIGHT_IP_ADDRESS)