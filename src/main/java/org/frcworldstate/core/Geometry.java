package org.frcworldstate.core;

/** SI units. Fixed BLUE_FIELD origin, +x down field, +y left, CCW radians. */
public final class Geometry {
    private Geometry() {}
    public static double finite(double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException("nonfinite value");
        return value;
    }
    public static double nonnegative(double value) {
        if (finite(value) < 0) throw new IllegalArgumentException("negative value");
        return value;
    }
    public static String id(String value) {
        if (value == null || value.isBlank()) throw new IllegalArgumentException("missing identity");
        return value;
    }
    public static long time(long value) {
        if (value < 0) throw new IllegalArgumentException("negative timestamp/id");
        return value;
    }
    public static double angle(double radians) {
        return Math.atan2(Math.sin(finite(radians)), Math.cos(radians));
    }
    public record Vec2(double x, double y) {
        public Vec2 { finite(x); finite(y); }
        public Vec2 add(Vec2 b) { return new Vec2(x+b.x,y+b.y); }
        public Vec2 subtract(Vec2 b) { return new Vec2(x-b.x,y-b.y); }
        public Vec2 scale(double n) { return new Vec2(x*n,y*n); }
        public double norm() { return Math.hypot(x,y); }
        public Vec2 rotate(double a) {
            return new Vec2(x*Math.cos(a)-y*Math.sin(a),x*Math.sin(a)+y*Math.cos(a));
        }
    }
    public record Pose2(Vec2 position, double headingRad) {
        public Pose2 { java.util.Objects.requireNonNull(position); headingRad=angle(headingRad); }
        public Vec2 toField(Vec2 robotRelative) { return position.add(robotRelative.rotate(headingRad)); }
        public Vec2 toRobot(Vec2 field) { return field.subtract(position).rotate(-headingRad); }
        public Pose2 interpolate(Pose2 b, double t) {
            return new Pose2(position.add(b.position.subtract(position).scale(t)),
                    headingRad+angle(b.headingRad-headingRad)*t);
        }
    }
    public record Velocity2(Vec2 linearMps, double angularRadPerSec) {
        public Velocity2 { java.util.Objects.requireNonNull(linearMps); finite(angularRadPerSec); }
        public static Velocity2 zero() { return new Velocity2(new Vec2(0,0),0); }
    }
    /** Explicit conservative covariance supplied by robot config; never fabricated estimator API data. */
    public record Uncertainty(double xxM2, double xyM2, double yyM2) {
        public Uncertainty {
            nonnegative(xxM2); finite(xyM2); nonnegative(yyM2);
            if (Math.abs(xyM2) > Math.sqrt(xxM2)*Math.sqrt(yyM2)*(1+1e-12)) throw new IllegalArgumentException("covariance not PSD");
        }
        public Uncertainty plus(Uncertainty b) { return new Uncertainty(xxM2+b.xxM2,xyM2+b.xyM2,yyM2+b.yyM2); }
        public Uncertainty grow(double variance) { return new Uncertainty(xxM2+nonnegative(variance),xyM2,yyM2+variance); }
        public Uncertainty rotate(double a) {
            double c=Math.cos(a),s=Math.sin(a);
            return new Uncertainty(c*c*xxM2-2*c*s*xyM2+s*s*yyM2,
                c*s*(xxM2-yyM2)+(c*c-s*s)*xyM2,s*s*xxM2+2*c*s*xyM2+c*c*yyM2);
        }
        public double maxVariance() { return (xxM2+yyM2+Math.hypot(xxM2-yyM2,2*xyM2))/2; }
    }
    public record FieldIdentity(String season, String mapId, String geometryRevision) {
        public FieldIdentity { id(season); id(mapId); id(geometryRevision); }
    }
    /** Call only at robot/config boundary. Core field observations must already use BLUE_FIELD. */
    public static Pose2 allianceToBlue(Pose2 alliancePose, boolean red, double fieldLengthM, double fieldWidthM) {
        nonnegative(fieldLengthM); nonnegative(fieldWidthM);
        return red ? new Pose2(new Vec2(fieldLengthM-alliancePose.position.x,fieldWidthM-alliancePose.position.y),
                alliancePose.headingRad+Math.PI) : alliancePose;
    }
}
