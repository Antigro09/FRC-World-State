package org.frcworldstate.core;

import java.util.*;
import java.util.function.UnaryOperator;
import static org.frcworldstate.core.Geometry.*;
import static org.frcworldstate.core.World.*;

/** Owner-thread bounded history. A nonempty estimator sample must never bypass these bounds. */
public final class PoseHistory {
    private final int capacity;
    private final long durationUs;
    private final NavigableMap<Long,EgoState> samples=new TreeMap<>();
    private long revision;
    public PoseHistory(int capacity,long durationUs) {
        if(capacity<2||capacity>10000||durationUs<=0) throw new IllegalArgumentException("history bounds");
        this.capacity=capacity; this.durationUs=durationUs;
    }
    public void add(EgoState ego) {
        if(!samples.isEmpty()&&ego.timeUs()<samples.lastKey()) throw new IllegalArgumentException("ego time went backwards");
        samples.put(ego.timeUs(),ego);
        while(samples.size()>capacity||(!samples.isEmpty()&&ego.timeUs()-samples.firstKey()>durationUs)) samples.pollFirstEntry();
    }
    public Optional<EgoState> sample(long captureUs) {
        if(samples.isEmpty()||captureUs<samples.firstKey()||captureUs>samples.lastKey()) return Optional.empty();
        var a=samples.floorEntry(captureUs); var b=samples.ceilingEntry(captureUs);
        if(a.getKey().equals(b.getKey())) return Optional.of(a.getValue());
        double t=(double)(captureUs-a.getKey())/(b.getKey()-a.getKey());
        EgoState x=a.getValue(),y=b.getValue();
        Uncertainty u=new Uncertainty(Math.max(x.uncertainty().maxVariance(),y.uncertainty().maxVariance()),0,
                Math.max(x.uncertainty().maxVariance(),y.uncertainty().maxVariance()));
        return Optional.of(new EgoState(captureUs,x.fieldPose().interpolate(y.fieldPose(),t),
                new Velocity2(x.fieldVelocity().linearMps().add(y.fieldVelocity().linearMps().subtract(x.fieldVelocity().linearMps()).scale(t)),
                    x.fieldVelocity().angularRadPerSec()+t*(y.fieldVelocity().angularRadPerSec()-x.fieldVelocity().angularRadPerSec())),u,
                x.localizationValid()&&y.localizationValid()));
    }
    /** Caller owns corrected poses. Velocity is kept authoritative; corrections never manufacture velocity. */
    public void correct(UnaryOperator<Pose2> correction) {
        NavigableMap<Long,EgoState> corrected=new TreeMap<>();
        for(var entry:samples.entrySet()) { EgoState e=entry.getValue();corrected.put(entry.getKey(),new EgoState(e.timeUs(),correction.apply(e.fieldPose()),e.fieldVelocity(),e.uncertainty(),e.localizationValid())); }
        samples.clear();samples.putAll(corrected);revision++;
    }
    public void replaceCorrectedHistory(List<EgoState> corrected) {
        if(corrected.size()!=samples.size())throw new IllegalArgumentException("correction requires entire retained history");
        for(EgoState e:corrected) {
            EgoState old=samples.get(e.timeUs());
            if(old==null||!old.fieldVelocity().equals(e.fieldVelocity())||!old.uncertainty().equals(e.uncertainty())
                    ||old.localizationValid()!=e.localizationValid())throw new IllegalArgumentException("correction may alter only retained poses");
        }
        if(corrected.stream().map(EgoState::timeUs).distinct().count()!=samples.size())throw new IllegalArgumentException("duplicate corrected time");
        for(EgoState e:corrected)samples.put(e.timeUs(),e);revision++;
    }
    public int capacity() { return capacity; }
    public long durationUs() { return durationUs; }
    public long revision() { return revision; }
    /** Integrate only authoritative ego velocity; disagreement suppresses inferred object velocity. */
    public boolean motionConsistent(long fromUs,long toUs,double positionToleranceM,double headingToleranceRad) {
        Optional<EgoState> from=sample(fromUs),to=sample(toUs);
        if(from.isEmpty()||to.isEmpty()||toUs<=fromUs)return false;
        List<EgoState> interval=new ArrayList<>();interval.add(from.get());
        for(EgoState e:samples.subMap(fromUs,false,toUs,false).values())interval.add(e);
        interval.add(to.get());Vec2 integrated=new Vec2(0,0);double heading=0;
        for(int i=1;i<interval.size();i++) {
            EgoState a=interval.get(i-1),b=interval.get(i);double dt=(b.timeUs()-a.timeUs())/1e6;
            integrated=integrated.add(a.fieldVelocity().linearMps().add(b.fieldVelocity().linearMps()).scale(dt/2));
            heading+=(a.fieldVelocity().angularRadPerSec()+b.fieldVelocity().angularRadPerSec())*dt/2;
        }
        return to.get().fieldPose().position().subtract(from.get().fieldPose().position()).subtract(integrated).norm()<=positionToleranceM
            &&Math.abs(angle(to.get().fieldPose().headingRad()-from.get().fieldPose().headingRad()-heading))<=headingToleranceRad;
    }
    public OptionalLong oldestUs() { return samples.isEmpty()?OptionalLong.empty():OptionalLong.of(samples.firstKey()); }
    public OptionalLong newestUs() { return samples.isEmpty()?OptionalLong.empty():OptionalLong.of(samples.lastKey()); }
    public List<EgoState> retained() { return List.copyOf(samples.values()); }
    public void clear() { samples.clear(); revision++; }
}
