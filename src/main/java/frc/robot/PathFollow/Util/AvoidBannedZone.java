
package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Translation2d;

public class AvoidBannedZone {
    private static RectanglePos[] bannedPos;

    public static boolean isInsideArc(Arc arc){
        for (Translation2d point : arc.getPoints()) {
            for(RectanglePos pos : bannedPos){
                if(pos.isInside(point)) return true;
            }
        }
        return false;
    }
    public static boolean isInsideLeg(Leg leg){
        Translation2d p1 = leg.p1;
        Translation2d p2 = leg.p2;
        for(RectanglePos pos : bannedPos){
            for(Translation2d[] line : pos.getLinesPoints()){
                if(isIntersecting(p1, p2, line[0], line[1])) return true;
            }
        }
        return false;
    }
    public static pathPoint[] fixPoint(){
        //TODO finish
        return new pathPoint[]{};
    }

    private static boolean isIntersecting(Translation2d legP1, Translation2d legP2, Translation2d recP1, Translation2d recP2){
        double slopeLeg = calcSlope(legP1, legP2);
        double bParamLeg = calcBParam(legP1, slopeLeg);
        double slopeRec = calcSlope(recP1, recP2);
        double bParamRec = calcBParam(recP1, slopeRec);

        if(slopeLeg == slopeRec) return false;
        double xIntersection = (bParamRec - bParamLeg) / (slopeRec - slopeLeg);
        return xIntersection >= legP1.getX() && xIntersection <= legP2.getX();
    }
    private static double calcSlope(Translation2d p1, Translation2d p2){
        return (p1.getX() - p2.getX()) / (p1.getY() - p2.getY());
    }
    private static double calcBParam(Translation2d p1, double slope){
        return -((slope * p1.getX()) - p1.getY());
    }
}
