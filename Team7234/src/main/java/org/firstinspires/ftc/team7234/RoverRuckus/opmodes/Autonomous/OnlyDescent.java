package org.firstinspires.ftc.team7234.RoverRuckus.opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team7234.RoverRuckus.common.AutoBase;
import org.firstinspires.ftc.team7234.RoverRuckus.common.enums.AllianceColor;
import org.firstinspires.ftc.team7234.RoverRuckus.common.enums.FieldPosition;

@Autonomous(name = "Only Descend", group = "Botman")
public class OnlyDescent extends AutoBase {
    public OnlyDescent(){
        super(AllianceColor.RED, FieldPosition.DEPOT, true);
    }
}
