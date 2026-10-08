package org.frcworldstate.core;

import java.util.List;
import java.util.Objects;
import static org.frcworldstate.core.Geometry.*;

/** Version 1 planner SPI. Implementations may depend on core; core never depends on implementations. */
@FunctionalInterface
public interface PlannerBackend {
    String CONTRACT_VERSION = "frc-planner/1";
    Result plan(Request request);
    @FunctionalInterface interface Cancellation { boolean cancelled(); }
    /** System.nanoTime domain, never robot/NT timestamp domain. Check before and during bounded search. */
    record SearchBudget(long startedNanos, long limitNanos) {
        public SearchBudget { if(limitNanos<=0) throw new IllegalArgumentException("budget must be positive"); }
        public boolean expired(long nowNanos) { return nowNanos-startedNanos>=limitNanos; }
    }
    record Goal(Pose2 pose, double positionToleranceM, double headingToleranceRad, double velocityToleranceMps) {
        public Goal { Objects.requireNonNull(pose); nonnegative(positionToleranceM); nonnegative(headingToleranceRad); nonnegative(velocityToleranceMps); }
    }
    record Footprint(double lengthM, double widthM) {
        public Footprint { if(nonnegative(lengthM)==0||nonnegative(widthM)==0) throw new IllegalArgumentException("empty footprint"); }
        public double boundingRadiusM() { return Math.hypot(lengthM,widthM)/2; }
    }
    record Constraints(double maxSpeedMps, double maxAccelerationMps2, double maxAngularSpeedRadPerSec) {
        public Constraints { if(nonnegative(maxSpeedMps)==0||nonnegative(maxAccelerationMps2)==0||nonnegative(maxAngularSpeedRadPerSec)==0) throw new IllegalArgumentException("zero constraint"); }
    }
    /** Conservative occupancy circle/envelope, valid across the entire request validity interval. */
    record Obstacle(String id, Vec2 centerM, double radiusM, double uncertaintyMarginM,
                    long validUntilUs, boolean dynamic) {
        public Obstacle { Geometry.id(id); Objects.requireNonNull(centerM); nonnegative(radiusM); nonnegative(uncertaintyMarginM); time(validUntilUs); }
        public double envelopeRadiusM() { return radiusM+uncertaintyMarginM; }
    }
    record Bounds(double minXM, double minYM, double maxXM, double maxYM) {
        public Bounds { finite(minXM); finite(minYM); finite(maxXM); finite(maxYM); if(minXM>=maxXM||minYM>=maxYM) throw new IllegalArgumentException("invalid bounds"); }
    }
    record Request(String requestId, String taskId, long epoch, long snapshotId,
            long obstacleMapVersion, FieldIdentity field, Pose2 startPose, Velocity2 startVelocity,
            Goal goal, Footprint footprint, Constraints constraints, Bounds bounds,
            List<Obstacle> obstacles, long issuedUs, long validUntilUs,
            SearchBudget budget, Cancellation cancellation) {
        public Request {
            id(requestId); id(taskId); time(epoch); time(snapshotId); time(obstacleMapVersion);
            Objects.requireNonNull(field); Objects.requireNonNull(startPose); Objects.requireNonNull(startVelocity);
            Objects.requireNonNull(goal); Objects.requireNonNull(footprint); Objects.requireNonNull(constraints); Objects.requireNonNull(bounds);
            obstacles=List.copyOf(obstacles); if(obstacles.size()>4096) throw new IllegalArgumentException("obstacle capacity exceeded");
            time(issuedUs); time(validUntilUs); if(validUntilUs<=issuedUs) throw new IllegalArgumentException("expired request");
            if(obstacles.stream().anyMatch(o->o.validUntilUs()<validUntilUs)) throw new IllegalArgumentException("obstacle envelope expires before request");
            Objects.requireNonNull(budget); Objects.requireNonNull(cancellation);
        }
    }
    enum Status { SUCCESS, NO_PATH, TIMEOUT, CANCELLED, INVALID_INPUT, STALE_RESULT }
    /** Geometry alone never completes a task and must be independently collision-checked while followed. */
    record GeometricPath(List<Pose2> points) {
        public GeometricPath { points=List.copyOf(points); if(points.size()<2||points.size()>10000) throw new IllegalArgumentException("invalid path size"); }
    }
    record TimedPoint(double timeSec, Pose2 pose, Velocity2 velocity) {
        public TimedPoint { nonnegative(timeSec); Objects.requireNonNull(pose); Objects.requireNonNull(velocity); }
    }
    record TimedTrajectory(List<TimedPoint> points) {
        public TimedTrajectory {
            points=List.copyOf(points); if(points.size()<2||points.size()>10000) throw new IllegalArgumentException("invalid trajectory size");
            double prev=-1; for(TimedPoint p:points) { if(p.timeSec()<=prev) throw new IllegalArgumentException("nonmonotonic trajectory"); prev=p.timeSec(); }
        }
    }
    record Result(String requestId, String taskId, long epoch, long snapshotId, long obstacleMapVersion,
            Status status, GeometricPath path, TimedTrajectory trajectory, long solverDurationNanos,
            long validUntilUs, String backendId, String detail) {
        public Result {
            id(requestId); id(taskId); time(epoch); time(snapshotId); time(obstacleMapVersion);
            Objects.requireNonNull(status); time(solverDurationNanos); time(validUntilUs); id(backendId); Objects.requireNonNull(detail);
            if(status==Status.SUCCESS && path==null) throw new IllegalArgumentException("success needs geometry");
            if(status!=Status.SUCCESS && (path!=null||trajectory!=null)) throw new IllegalArgumentException("failure carries path");
        }
        public boolean matches(Request r, World.WorldSnapshot s, long nowUs) {
            return requestId.equals(r.requestId())&&taskId.equals(r.taskId())&&epoch==r.epoch()&&epoch==s.epoch()
                &&snapshotId==r.snapshotId()&&snapshotId==s.id()&&obstacleMapVersion==r.obstacleMapVersion()
                &&obstacleMapVersion==s.obstacleMapVersion()&&nowUs>=r.issuedUs()&&nowUs<=validUntilUs&&validUntilUs<=r.validUntilUs();
        }
    }
}
