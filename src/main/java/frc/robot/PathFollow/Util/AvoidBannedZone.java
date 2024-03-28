
package frc.robot.PathFollow.Util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.PathFollow.PathFollowConstants.*;

public class AvoidBannedZone {
    private static RectanglePos[] bannedPos;
    private static RectanglePos intersectionPos = null;

    private static enum rectanglePoints{
        TOP_LEFT, TOP_RIGHT, BOTTOM_RIGHT, BOTTOM_LEFT;
    }
    

    private static boolean isInsideArc(Arc arc){
        intersectionPos = null;
        for (Translation2d point : arc.getPoints()) {
            for(RectanglePos pos : bannedPos){
                intersectionPos = pos;
                
                if(pos.isInside(point)) return true;
            }
        }
        return false;
    }
    private static boolean isInsideLeg(Leg leg){
        intersectionPos = null;
        Translation2d p1 = leg.p1;
        Translation2d p2 = leg.p2;
        for(RectanglePos pos : bannedPos){
            for(Translation2d[] line : pos.getLinesPoints()){
                
                if(isIntersecting(p1, p2, line[0], line[1])){
                    intersectionPos = pos;
                    return true;
                } 
            }
        }
        return false;
    }
    public static pathPoint[] fixPoint(Segment segment, Translation2d pose){
        List<pathPoint> newPoints = new ArrayList<pathPoint>();
        
        //checks if segment given is leg or arc
        boolean isLeg = segment instanceof Leg;
        
        if(intersectionPos == null) return new pathPoint[]{};
        if(isLeg){
            newPoints.add(new pathPoint(segment.p1, Rotation2d.fromDegrees(0), 0, PATH_MAX_VELOCITY_AVOID));
            if(isInsideLeg((Leg) segment)){
                Pair<Translation2d, Translation2d> points = getFixingPoints(intersectionPos, segment.p1, segment.p2, pose);
                newPoints.add(new pathPoint(points.getFirst(), Rotation2d.fromDegrees(0), 1, PATH_MAX_VELOCITY_AVOID));
                newPoints.add(new pathPoint(points.getSecond(), Rotation2d.fromDegrees(0), 1, PATH_MAX_VELOCITY));  
            }
            newPoints.add(new pathPoint(segment.p2, Rotation2d.fromDegrees(0), PATH_MAX_VELOCITY_AVOID));
        }

        return (pathPoint[]) newPoints.toArray();

    }
    private static Pair<Translation2d, Translation2d> getFixingPoints(RectanglePos pos, Translation2d p1, Translation2d p2, Translation2d pose){

        //TODO FINISH LOGIC
        Translation2d firstPoint = new Translation2d();
        Translation2d secondPoint = new Translation2d();

        Translation2d topLtoBottomR = pos.getTopLeft().minus(pos.getBottomRight());
        Translation2d topRtoBottomL = pos.getTopRight().minus(pos.getBottomLeft());

        Translation2d topLpoint = topLtoBottomR.plus(new Translation2d(PATH_MIN_DISTANCE_FROM_CORNER, topLtoBottomR.getAngle()));
        Translation2d topRpoint = topRtoBottomL.plus(new Translation2d(PATH_MIN_DISTANCE_FROM_CORNER, topRtoBottomL.getAngle()));
        Translation2d bottomLpoint = topLtoBottomR.minus(new Translation2d(PATH_MIN_DISTANCE_FROM_CORNER, topLtoBottomR.getAngle()));
        Translation2d bottomRpoint = topLtoBottomR.minus(new Translation2d(PATH_MIN_DISTANCE_FROM_CORNER, topRtoBottomL.getAngle()));

        Translation2d[] fixingPoints = {topLpoint, topRpoint, bottomLpoint, bottomRpoint};

        firstPoint = calcClosetPoint(fixingPoints, pose);
        secondPoint = calcClosetPoint(fixingPoints, firstPoint);

        
        





    }

    private static Translation2d calcClosetPoint(Translation2d[] points, Translation2d pose){
        Translation2d closet = new Translation2d(Integer.MAX_VALUE, Integer.MAX_VALUE);
        for (Translation2d current : points) {
            if(pose.getDistance(current) < pose.getDistance(closet)) closet = current;
        }
        return closet;
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
