package org.frcworldstate.core;

import static org.frcworldstate.core.Geometry.*;
import static org.frcworldstate.core.PlannerBackend.*;

/** Optional OFF-CONTROL-PATH result validation. Bounded by the request search budget and cancellation. */
public final class PlannerValidation {
    private PlannerValidation() {}
    public static Result validate(Request request, Result result, World.WorldSnapshot originalSnapshot,
            long currentEpoch,long currentMapVersion,long nowUs) {
        if(result==null)return reject(request,Status.INVALID_INPUT,"backend returned null");
        if(request.cancellation().cancelled())return reject(request,Status.CANCELLED,"cancelled");
        if(!request.field().equals(originalSnapshot.field())||!result.matches(request,originalSnapshot,nowUs)||currentEpoch!=request.epoch()||currentMapVersion!=request.obstacleMapVersion())
            return reject(request,Status.STALE_RESULT,"request/snapshot/epoch/map/expiry mismatch");
        if(result.status()!=Status.SUCCESS)return result;
        if(result.trajectory()!=null)return reject(request,Status.INVALID_INPUT,"timed trajectory requires a separate qualified validator");
        if(result.solverDurationNanos()>request.budget().limitNanos())return reject(request,Status.TIMEOUT,"reported solver duration exceeds request budget");
        var points=result.path().points();
        Pose2 first=points.get(0),last=points.get(points.size()-1);
        if(first.position().subtract(request.startPose().position()).norm()>1e-6||Math.abs(angle(first.headingRad()-request.startPose().headingRad()))>1e-6
            ||last.position().subtract(request.goal().pose().position()).norm()>request.goal().positionToleranceM()
            ||Math.abs(angle(last.headingRad()-request.goal().pose().headingRad()))>request.goal().headingToleranceRad())
            return reject(request,Status.INVALID_INPUT,"path endpoints do not satisfy request");
        double robot=request.footprint().boundingRadiusM();
        for(int i=0;i<points.size();i++) {
            if(request.cancellation().cancelled())return reject(request,Status.CANCELLED,"cancelled during validation");
            if(request.budget().expired(System.nanoTime()))return reject(request,Status.TIMEOUT,"validation exceeded total search budget");
            Vec2 p=points.get(i).position(); Bounds b=request.bounds();
            if(p.x()-robot<b.minXM()||p.y()-robot<b.minYM()||p.x()+robot>b.maxXM()||p.y()+robot>b.maxYM())
                return reject(request,Status.INVALID_INPUT,"footprint outside map bounds");
            Vec2 a=i==0?p:points.get(i-1).position();
            for(int j=0;j<request.obstacles().size();j++) {
                if((j&63)==0) {
                    if(request.cancellation().cancelled())return reject(request,Status.CANCELLED,"cancelled during validation");
                    if(request.budget().expired(System.nanoTime()))return reject(request,Status.TIMEOUT,"validation exceeded total budget");
                }
                Obstacle o=request.obstacles().get(j);
                if(distanceToSegment(o.centerM(),a,p)<=robot+o.envelopeRadiusM())
                    return reject(request,Status.INVALID_INPUT,"path intersects conservative obstacle envelope: "+o.id());
            }
        }
        return result;
    }
    public static double distanceToSegment(Vec2 point,Vec2 a,Vec2 b) {
        Vec2 d=b.subtract(a);double n=d.x()*d.x()+d.y()*d.y();
        if(n==0)return point.subtract(a).norm();
        Vec2 q=point.subtract(a);double t=Math.max(0,Math.min(1,(q.x()*d.x()+q.y()*d.y())/n));
        return point.subtract(a.add(d.scale(t))).norm();
    }
    private static Result reject(Request r,Status status,String detail) {
        return new Result(r.requestId(),r.taskId(),r.epoch(),r.snapshotId(),r.obstacleMapVersion(),status,null,null,0,
            r.validUntilUs(),"core-validator",detail);
    }
}
