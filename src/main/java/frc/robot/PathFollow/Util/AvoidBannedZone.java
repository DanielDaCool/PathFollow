
package frc.robot.PathFollow.Util;

import static frc.robot.PathFollow.PathFollowConstants.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AvoidBannedZone {
    private static RectanglePos[] bannedPos;
    private static RectanglePos intersectionPos = null;
    private static Segment currentSegment;

    

    private static boolean isInsideArc(Arc arc){
        for (Translation2d point : arc.getPoints()) {
            for(RectanglePos pos : bannedPos){
                intersectionPos = pos;
                
                if(pos.isInside(point)) return true;
            }
        }
        return false;
    }
    private static boolean isInsideLeg(Leg leg){
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
    public static Segment[] fixPoint(Segment segment, Translation2d pose){
        List<pathPoint> newPoints = new ArrayList<pathPoint>();
        
        //checks if segment given is leg or arc
        boolean isLeg = segment instanceof Leg;
        currentSegment = segment;
        
        
        
        if(isLeg){
            newPoints.add(new pathPoint(segment.p1, Rotation2d.fromDegrees(0), 0, PATH_MAX_VELOCITY_AVOID));
            if(isInsideLeg((Leg) segment)){
                Pair<Translation2d, Translation2d> points = getFixingPoints(intersectionPos, segment.p1, segment.p2, pose);
                newPoints.add(new pathPoint(points.getFirst(), Rotation2d.fromDegrees(0), 1, PATH_MAX_VELOCITY_AVOID));
                newPoints.add(new pathPoint(points.getSecond(), Rotation2d.fromDegrees(0), 1, PATH_MAX_VELOCITY_AVOID));  
            }
            newPoints.add(new pathPoint(segment.p2, Rotation2d.fromDegrees(0), PATH_MAX_VELOCITY_AVOID));
        }
        else{
            newPoints.add(new pathPoint(segment.p1, Rotation2d.fromDegrees(0), PATH_MAX_VELOCITY_AVOID));
            newPoints.add(new pathPoint(segment.p2, Rotation2d.fromDegrees(0), PATH_MAX_VELOCITY_AVOID));
            if(isInsideArc((Arc)segment)) SmartDashboard.putBoolean("isValidSegment", false);
        }
        if(intersectionPos == null) return new Segment[] {segment};

        

        RoundedPoint calcFirstArc = new RoundedPoint(newPoints.get(0), newPoints.get(1), newPoints.get(2));
        RoundedPoint calcSecondArc = new RoundedPoint(newPoints.get(1), newPoints.get(2), newPoints.get(3));

        return new Segment[] {calcFirstArc.getArc(), calcSecondArc.getArc(), new Leg(calcSecondArc.endRange(), segment.p2)};

    }
    private static Pair<Translation2d, Translation2d> getFixingPoints(RectanglePos pos, Translation2d p1, Translation2d p2, Translation2d pose){

        

       
        
        Translation2d topLpoint = pos.getTopLeft().plus(new Translation2d(1, Rotation2d.fromDegrees(-135)));
        Translation2d topRpoint = pos.getTopRight().plus(new Translation2d(1, Rotation2d.fromDegrees(135)));
        Translation2d bottomLpoint = pos.getBottomLeft().plus(new Translation2d(1, Rotation2d.fromDegrees(45)));
        Translation2d bottomRpoint = pos.getBottomRight().plus(new Translation2d(1, Rotation2d.fromDegrees(-45)));

        topLpoint = topLpoint.times(PATH_MIN_DISTANCE_FROM_CORNER);
        topRpoint = topRpoint.times(PATH_MIN_DISTANCE_FROM_CORNER);
        bottomLpoint = bottomLpoint.times(PATH_MIN_DISTANCE_FROM_CORNER);
        bottomRpoint = bottomRpoint.times(PATH_MIN_DISTANCE_FROM_CORNER);



        

        

        HashMap<Translation2d, Translation2d[]> points = new HashMap<>();
        points.put(topLpoint, new Translation2d[] {topRpoint, bottomLpoint});
        points.put(topRpoint, new Translation2d[] {topLpoint, bottomRpoint});
        points.put(bottomRpoint, new Translation2d[] {topRpoint, bottomLpoint});
        points.put(bottomLpoint, new Translation2d[] {topLpoint, bottomRpoint});
        
        Translation2d[] fixingPoints = {topLpoint, topRpoint, bottomLpoint, bottomRpoint};

        Translation2d firstPoint = calcClosetPoint(fixingPoints, pose);

        Translation2d secondPoint = calcClosetPoint(points.get(firstPoint), currentSegment.p2);
        System.out.println("FIRST: " + firstPoint);
        System.out.println("SECOND: " + secondPoint);


      return new Pair<Translation2d,Translation2d>(firstPoint, secondPoint);
     


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