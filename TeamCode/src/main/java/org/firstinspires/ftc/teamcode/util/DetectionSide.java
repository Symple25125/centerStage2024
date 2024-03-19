package org.firstinspires.ftc.teamcode.util;

public enum DetectionSide {
    CLOSE,
    FAR,
    CENTER;

    public DetectionSide mirror() {
        if(this == CLOSE) return FAR;
        if(this == FAR) return CLOSE;
        return CENTER;
    }
}
