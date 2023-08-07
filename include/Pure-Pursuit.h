#pragma once

class PurePursuitClass{
    public:
    void PurePursuit();

    double DesiredX, DesiredY, DesiredHeading;
    double ChangeinX, ChangeinY;
    double TargetThetaRad, TargetThetaDeg, TargetDistance;
    double IotaSquared, Gamma;
    double TurnAngle;

};