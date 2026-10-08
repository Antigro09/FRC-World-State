package org.frcworldstate.core;

import java.util.*;
import java.util.function.UnaryOperator;
import static org.frcworldstate.core.Geometry.*;
import static org.frcworldstate.core.World.*;

/** Deterministic owner-thread tracker. Empty frames never remove occupancy or prove free space. */
public final class WorldEngine {
    public record Config(int maxTracks,int rawCapacity,int snapshotCapacity,long maxFrameAgeUs,
            long expiryUs,long correlationWindowUs,long minVelocityDtUs,double associationM,
            double minConfidence,double maxMeasurementVarianceM2,double processVariancePerSec,
            double maxObjectSpeedMps,int confirmMeasurements,double headingErrorBoundRad,double maxObservationRangeM,double mapMovementThresholdM,double poseInnovationLimitM,double headingInnovationLimitRad,double velocityConsistencyM,boolean enableConstantVelocity) {
        public Config {
            if(maxTracks<1||maxTracks>256||rawCapacity<1||rawCapacity>4096||snapshotCapacity<1||snapshotCapacity>1024
                ||maxFrameAgeUs<1||expiryUs<1||correlationWindowUs<0||minVelocityDtUs<1||confirmMeasurements<1)
                throw new IllegalArgumentException("invalid tracker bounds");
            if(nonnegative(associationM)==0||nonnegative(maxMeasurementVarianceM2)==0||nonnegative(maxObjectSpeedMps)==0)
                throw new IllegalArgumentException("invalid tracker gate");
            nonnegative(minConfidence); if(minConfidence>1) throw new IllegalArgumentException("confidence > 1");
            nonnegative(processVariancePerSec); nonnegative(headingErrorBoundRad);
            if(headingErrorBoundRad>Math.PI||nonnegative(maxObservationRangeM)==0||nonnegative(mapMovementThresholdM)==0)
                throw new IllegalArgumentException("invalid projection/map bound");
            if(nonnegative(poseInnovationLimitM)==0||nonnegative(headingInnovationLimitRad)==0||headingInnovationLimitRad>Math.PI
                ||nonnegative(velocityConsistencyM)==0)throw new IllegalArgumentException("invalid pose discontinuity limits");
        }
    }
    public enum FrameStatus { ACCEPTED, WRONG_EPOCH, FUTURE, STALE, HISTORY_BOUNDS, INVALID_LOCALIZATION,
        DUPLICATE, OUT_OF_ORDER, SOURCE_RESET, CAPACITY }
    public record FrameDecision(long decisionId,FrameStatus status,int acceptedMeasurements,long epoch,String reason) {}
    public sealed interface ReplayInput permits EgoInput,FrameInput,PoseCorrectionInput,ResetInput,SnapshotInput {}
    public record EgoInput(EgoState ego) implements ReplayInput {}
    public record FrameInput(ObservationFrame frame,FrameStatus status,int acceptedMeasurements) implements ReplayInput {}
    public record PoseCorrectionInput(List<EgoState> correctedHistory) implements ReplayInput {
        public PoseCorrectionInput { correctedHistory=List.copyOf(correctedHistory); }
    }
    public record SnapshotInput(WorldSnapshot snapshot) implements ReplayInput {}
    public record ResetInput(ResetReason reason,FieldIdentity field,boolean clearPoseHistory,boolean derived) implements ReplayInput {}
    public record InputEvent(long decisionId,String type,long epoch,long timeUs,String identity,ReplayInput input) {}
    public record RetainedObservation(long trackId,SourceStamp stamp,Observation observation) {}
    private record SourceState(String boot,String calibration,String mount,long sequence,long captureUs) {}
    private static final class Track {
        long id,lastUs,velocityTrustedAfterUs; int measurements;
        String cls; Vec2 p,v=new Vec2(0,0); double confidence; Uncertainty u; SourceStamp stamp;
        final Map<String,Long> correlationTimes=new HashMap<>();
        final Deque<SourceStamp> provenance=new ArrayDeque<>();
    }
    private final Config config;
    private final PoseHistory poses;
    private final NavigableMap<Long,Track> tracks=new TreeMap<>();
    private final Map<String,SourceState> sources=new HashMap<>();
    private final Deque<RetainedObservation> raw=new ArrayDeque<>();
    private final Deque<WorldSnapshot> snapshots=new ArrayDeque<>();
    private final Deque<InputEvent> log=new ArrayDeque<>();
    private FieldIdentity field;
    private long epoch,trackCounter,snapshotCounter,mapVersion,decisionCounter,lastCaptureUs=-1;
    public WorldEngine(Config config,PoseHistory poses,FieldIdentity field) {
        this.config=Objects.requireNonNull(config); this.poses=Objects.requireNonNull(poses); this.field=Objects.requireNonNull(field);
    }
    public long epoch() { return epoch; }
    public PoseHistory poseHistory() { return poses; }
    public Config config() { return config; }
    public void addEgo(EgoState e) { addEgoInternal(e);event(++decisionCounter,"EGO",e.timeUs(),"robot-authoritative",new EgoInput(e)); }
    private void addEgoInternal(EgoState e) {
        OptionalLong last=poses.newestUs();
        if(last.isPresent()&&e.timeUs()==last.getAsLong()
                &&!e.fieldPose().equals(poses.sample(last.getAsLong()).orElseThrow().fieldPose()))
            resetInternal(ResetReason.LOCALIZATION_HARD_RESET,field,true,e.timeUs(),true);
        else if(last.isPresent()&&e.timeUs()>last.getAsLong()) {
            EgoState prev=poses.sample(last.getAsLong()).orElseThrow();double dt=(e.timeUs()-prev.timeUs())/1e6;
            Vec2 expected=prev.fieldVelocity().linearMps().add(e.fieldVelocity().linearMps()).scale(dt/2);
            double heading=(prev.fieldVelocity().angularRadPerSec()+e.fieldVelocity().angularRadPerSec())*dt/2;
            if(e.timeUs()-prev.timeUs()>poses.durationUs()||e.fieldPose().position().subtract(prev.fieldPose().position()).subtract(expected).norm()>config.poseInnovationLimitM()
                    ||Math.abs(angle(e.fieldPose().headingRad()-prev.fieldPose().headingRad()-heading))>config.headingInnovationLimitRad())
                resetInternal(ResetReason.LOCALIZATION_HARD_RESET,field,true,e.timeUs(),true);
        }
        poses.add(e);
    }
    private FrameDecision decision(ObservationFrame frame,FrameStatus s,int n,long now,String reason) {
        long id=++decisionCounter; event(id,"FRAME_"+s,now,reason,new FrameInput(frame,s,n)); return new FrameDecision(id,s,n,epoch,reason);
    }
    private void event(long id,String type,long now,String identity,ReplayInput input) {
        log.addLast(new InputEvent(id,type,epoch,now,identity,input)); while(log.size()>config.rawCapacity())log.removeFirst();
    }
    public FrameDecision accept(ObservationFrame frame,long nowUs) {
        time(nowUs); SourceStamp s=frame.stamp();
        if(frame.epoch()!=epoch)return decision(frame,FrameStatus.WRONG_EPOCH,0,nowUs,s.sourceId());
        if(s.captureUs()>nowUs||s.publicationUs()>nowUs)return decision(frame,FrameStatus.FUTURE,0,nowUs,s.sourceId());
        if(nowUs-s.captureUs()>config.maxFrameAgeUs())return decision(frame,FrameStatus.STALE,0,nowUs,s.sourceId());
        SourceState prev=sources.get(s.sourceId());
        if(prev!=null&&(!prev.boot.equals(s.bootId())||!prev.calibration.equals(s.calibrationRevision())||!prev.mount.equals(s.mountRevision()))) {
            resetInternal(ResetReason.RECONNECT,field,false,nowUs,true);
            return decision(frame,FrameStatus.SOURCE_RESET,0,nowUs,"source/revision changed; resubmit in new epoch");
        }
        if(prev!=null&&s.sequence()==prev.sequence)return decision(frame,FrameStatus.DUPLICATE,0,nowUs,s.sourceId());
        if(prev!=null&&(s.sequence()<prev.sequence||s.captureUs()<=prev.captureUs))return decision(frame,FrameStatus.OUT_OF_ORDER,0,nowUs,s.sourceId());
        Optional<EgoState> sample=poses.sample(s.captureUs());
        if(sample.isEmpty())return decision(frame,FrameStatus.HISTORY_BOUNDS,0,nowUs,s.sourceId());
        if(!sample.get().localizationValid())return decision(frame,FrameStatus.INVALID_LOCALIZATION,0,nowUs,s.sourceId());
        if(s.captureUs()<lastCaptureUs)return decision(frame,FrameStatus.OUT_OF_ORDER,0,nowUs,"global capture high-water; reorder before core");
        if(!sources.containsKey(s.sourceId())&&sources.size()>=32)return decision(frame,FrameStatus.CAPACITY,0,nowUs,"source capacity");
        lastCaptureUs=Math.max(lastCaptureUs,s.captureUs());
        sources.put(s.sourceId(),new SourceState(s.bootId(),s.calibrationRevision(),s.mountRevision(),s.sequence(),s.captureUs()));
        expire(nowUs);
        int accepted=0; boolean mapChanged=false; Set<Long> used=new HashSet<>();
        List<Observation> ordered=frame.observations().stream().sorted(Comparator.comparing(Observation::detectionId)).toList();
        Set<String> detectionIds=new HashSet<>();
        for(Observation o:ordered) {
            if(!detectionIds.add(o.detectionId())||o.confidence()<config.minConfidence()
                    ||o.relativeUncertainty().maxVariance()>config.maxMeasurementVarianceM2()
                    ||o.robotRelativeM().norm()>config.maxObservationRangeM())continue;
            Vec2 measured=sample.get().fieldPose().toField(o.robotRelativeM());
            Uncertainty uncertainty=projectedUncertainty(o,sample.get());
            Track best=null; double bestD=Double.POSITIVE_INFINITY,secondD=Double.POSITIVE_INFINITY;
            for(Track t:tracks.values()) {
                if(used.contains(t.id)||!t.cls.equals(o.objectClass()))continue;
                double dt=Math.max(0,(s.captureUs()-t.lastUs)/1e6);
                double d=measured.subtract(t.p.add(t.v.scale(dt))).norm();
                // Conservative uncertainty-aware distance gate, plus a fixed physical association ceiling.
                double gate=Math.min(config.associationM(),3*Math.sqrt(uncertainty.maxVariance()+t.u.maxVariance())+0.05);
                if(d<=gate) {if(d<bestD) {secondD=bestD;bestD=d;best=t;}else secondD=Math.min(secondD,d);}
            }
            // Ambiguous crossings keep old identities and skip this measurement, rather than switch identity.
            if(best!=null&&secondD-bestD<0.05)continue;
            if(best!=null) {
                if(s.captureUs()<=best.lastUs)continue;
                Long correlated=best.correlationTimes.get(s.correlationGroup());
                if(correlated!=null&&s.captureUs()-correlated<=config.correlationWindowUs())continue;
                best.correlationTimes.values().removeIf(t->s.captureUs()-t>config.correlationWindowUs());
                if(!best.correlationTimes.containsKey(s.correlationGroup())&&best.correlationTimes.size()>=32)continue;
            } else {
                if(tracks.size()>=config.maxTracks())continue;
                best=new Track();best.id=++trackCounter;best.cls=o.objectClass();best.velocityTrustedAfterUs=-1;
                tracks.put(best.id,best);mapChanged=true;
            }
            if(best.measurements>0&&(measured.subtract(best.p).norm()>config.mapMovementThresholdM()
                    ||Math.abs(uncertainty.maxVariance()-best.u.maxVariance())>config.mapMovementThresholdM()*config.mapMovementThresholdM()))mapChanged=true;
            if(config.enableConstantVelocity()&&best.measurements>0&&best.lastUs>=best.velocityTrustedAfterUs&&s.captureUs()-best.lastUs>=config.minVelocityDtUs()
                    &&poses.motionConsistent(best.lastUs,s.captureUs(),config.velocityConsistencyM(),config.headingInnovationLimitRad())) {
                Vec2 v=measured.subtract(best.p).scale(1e6/(s.captureUs()-best.lastUs));
                best.v=v.norm()<=config.maxObjectSpeedMps()?v:new Vec2(0,0);
            } else best.v=new Vec2(0,0);
            best.p=measured;best.lastUs=s.captureUs();best.confidence=o.confidence();best.u=uncertainty;best.stamp=s;best.measurements++;
            best.correlationTimes.put(s.correlationGroup(),s.captureUs());best.provenance.addLast(s);while(best.provenance.size()>8)best.provenance.removeFirst();
            used.add(best.id);accepted++;
            raw.addLast(new RetainedObservation(best.id,s,o)); while(raw.size()>config.rawCapacity())raw.removeFirst();
        }
        if(mapChanged)mapVersion++;
        return decision(frame,FrameStatus.ACCEPTED,accepted,nowUs,s.sourceId()+":"+s.bootId()+":"+s.sequence());
    }
    private Uncertainty projectedUncertainty(Observation observation,EgoState ego) {
        double headingDisplacementBound=2*observation.robotRelativeM().norm()*Math.sin(config.headingErrorBoundRad()/2);
        return observation.relativeUncertainty().rotate(ego.fieldPose().headingRad()).plus(ego.uncertainty())
            .grow(headingDisplacementBound*headingDisplacementBound);
    }
    private void expire(long now) {
        int size=tracks.size(); tracks.values().removeIf(t->now-t.lastUs>config.expiryUs());
        if(tracks.size()!=size)mapVersion++;
        raw.removeIf(r->now-r.stamp.captureUs()>config.expiryUs()||!tracks.containsKey(r.trackId()));
    }
    public WorldSnapshot publish(EgoState ego) {
        addEgoInternal(ego);expire(ego.timeUs()); List<ObjectTrack> out=new ArrayList<>();
        for(Track t:tracks.values()) {
            double dt=(ego.timeUs()-t.lastUs)/1e6;
            if(dt<0)throw new IllegalArgumentException("snapshot before measurement");
            out.add(new ObjectTrack(t.id,epoch,t.cls,t.p.add(t.v.scale(dt)),t.v,ego.timeUs(),t.lastUs,t.confidence,
                t.u.grow(config.processVariancePerSec()*dt),t.measurements<config.confirmMeasurements()?TrackLifecycle.TENTATIVE:
                    (dt>0?TrackLifecycle.COASTING:TrackLifecycle.CONFIRMED),
                dt>0?EstimateKind.PROPAGATED:EstimateKind.MEASURED,List.copyOf(t.provenance)));
        }
        WorldSnapshot s=new WorldSnapshot(++snapshotCounter,epoch,mapVersion,field,ego,out);
        snapshots.addLast(s);while(snapshots.size()>config.snapshotCapacity())snapshots.removeFirst();event(++decisionCounter,"SNAPSHOT",ego.timeUs(),Long.toString(s.id()),new SnapshotInput(s));return s;
    }
    /** Reprojects retained raw relative measurements; no ego correction is interpreted as object motion. */
    public void correctPoses(UnaryOperator<Pose2> correction,long nowUs) {
        poses.correct(correction);reproject(nowUs);
    }
    /** Absolute corrected history supports deterministic replay without serializing a function. */
    public void applyCorrectedHistory(List<EgoState> corrected,long nowUs) { poses.replaceCorrectedHistory(corrected);reproject(nowUs); }
    private void reproject(long nowUs) {
        Set<Long> rebuilt=new HashSet<>();
        for(RetainedObservation r:raw) {
            Track t=tracks.get(r.trackId());Optional<EgoState> ego=poses.sample(r.stamp.captureUs());
            if(t==null||ego.isEmpty())continue;
            t.p=ego.get().fieldPose().toField(r.observation.robotRelativeM());t.lastUs=r.stamp.captureUs();
            t.u=projectedUncertainty(r.observation,ego.get());
            t.v=new Vec2(0,0);t.velocityTrustedAfterUs=poses.newestUs().orElse(nowUs);rebuilt.add(t.id);
        }
        tracks.keySet().removeIf(id->!rebuilt.contains(id)); mapVersion++; snapshots.clear();
        event(++decisionCounter,"POSE_CORRECTION",nowUs,Long.toString(poses.revision()),new PoseCorrectionInput(poses.retained()));
    }
    /** All old requests/goals/results become invalid by epoch. Controller and workers must observe/cancel. */
    public void reset(ResetReason reason,FieldIdentity newField,boolean clearPoseHistory,long nowUs) {
        resetInternal(reason,newField,clearPoseHistory,nowUs,false);
    }
    private void resetInternal(ResetReason reason,FieldIdentity newField,boolean clearPoseHistory,long nowUs,boolean derived) {
        Objects.requireNonNull(reason);Objects.requireNonNull(newField);time(nowUs);
        epoch++;mapVersion++;lastCaptureUs=-1;tracks.clear();raw.clear();sources.clear();snapshots.clear();field=Objects.requireNonNull(newField);
        if(clearPoseHistory)poses.clear();event(++decisionCounter,"RESET_"+reason,nowUs,field.mapId(),new ResetInput(reason,field,clearPoseHistory,derived));
    }
    public List<WorldSnapshot> history() { return List.copyOf(snapshots); }
    public List<RetainedObservation> rawHistory() { return List.copyOf(raw); }
    public List<InputEvent> decisionLog() { return List.copyOf(log); }
}
