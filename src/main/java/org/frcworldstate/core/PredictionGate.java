package org.frcworldstate.core;

import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import static org.frcworldstate.core.Predictor.*;
import static org.frcworldstate.core.Geometry.*;

/** Bounded, deterministic consumption gate. Rejection preserves the caller's deterministic fallback. */
public final class PredictionGate {
    public enum Status { ACCEPTED, ABSENT, CANCELLED, UNSUPPORTED_SCHEMA, MISMATCHED_IDENTITY,
        STALE_REQUEST, STALE_REPLY, FUTURE_REPLY, INVALID_INPUT, NONFINITE, INVALID_MASK,
        INVALID_UNCERTAINTY, OUT_OF_DOMAIN, DIMENSION_MISMATCH, UNSYNCHRONIZED, UNCALIBRATED }
    public record Decision(Status status, String detail, Reply acceptedReply) {
        public Decision { Objects.requireNonNull(status); Objects.requireNonNull(detail); if((status==Status.ACCEPTED)!=(acceptedReply!=null)) throw new IllegalArgumentException("decision reply mismatch"); }
        public boolean usesBaseline() { return status!=Status.ACCEPTED; }
    }
    public record MechanismLimit(String unit, double min, double max) {
        public MechanismLimit { id(unit); finite(min); finite(max); if(!Set.of("m","m/s","rad","rad/s","ratio","bool").contains(unit)||min>max) throw new IllegalArgumentException("mechanism unit/bound"); }
    }
    /** Limits require robot/model measurements before hardware use; these values do not certify safety. */
    public record Policy(Map<String,Set<String>> modelCalibrations, Set<FieldIdentity> allowedFields,
                         Set<String> allowedConfigurationRevisions,
                         double minXM, double minYM, double maxXM, double maxYM,
                         double maxSpeedMps, double maxOmegaRadps, double minVarianceM2,
                         double maxVarianceM2, double maxInitialOffsetM, long maxHorizonUs, long maxHistoryAgeUs,
                         long maxSyncErrorUs, Map<String,MechanismLimit> mechanisms) {
        public Policy {
            var copied=new java.util.HashMap<String,Set<String>>();
            modelCalibrations.forEach((model,calibrations)->{ id(model); calibrations.forEach(Geometry::id); copied.put(model,Set.copyOf(calibrations)); });
            modelCalibrations=Map.copyOf(copied); allowedFields=Set.copyOf(allowedFields); allowedConfigurationRevisions=Set.copyOf(allowedConfigurationRevisions); mechanisms=Map.copyOf(mechanisms);
            finite(minXM); finite(minYM); finite(maxXM); finite(maxYM);
            nonnegative(maxSpeedMps); nonnegative(maxOmegaRadps); nonnegative(minVarianceM2); nonnegative(maxVarianceM2); nonnegative(maxInitialOffsetM);
            time(maxHorizonUs); time(maxHistoryAgeUs); time(maxSyncErrorUs);
            if(modelCalibrations.isEmpty()||allowedFields.isEmpty()||allowedConfigurationRevisions.isEmpty()||minXM>=maxXM||minYM>=maxYM||maxSpeedMps==0||maxOmegaRadps==0||minVarianceM2==0||minVarianceM2>maxVarianceM2||maxHorizonUs==0||maxHistoryAgeUs==0) throw new IllegalArgumentException("invalid domain policy");
        }
    }
    private final Policy policy;
    private record Pair(String candidateId,String targetId) {}
    public PredictionGate(Policy policy) { this.policy=Objects.requireNonNull(policy); }
    private static Decision reject(Status status,String detail) { return new Decision(status,detail,null); }
    private static boolean finiteValues(double... a) { for(double v:a) if(!Double.isFinite(v)) return false; return true; }
    private boolean inside(double x,double y) { return x>=policy.minXM()&&x<=policy.maxXM()&&y>=policy.minYM()&&y<=policy.maxYM(); }
    private Status action(BodyTwist twist,java.util.List<MechanismValue> values) {
        if(!finiteValues(twist.vxMps(),twist.vyMps(),twist.omegaRadps())) return Status.NONFINITE;
        if(Math.hypot(twist.vxMps(),twist.vyMps())>policy.maxSpeedMps()||Math.abs(twist.omegaRadps())>policy.maxOmegaRadps()) return Status.OUT_OF_DOMAIN;
        Set<String> channels=new HashSet<>();
        for(MechanismValue v:values) {
            if(!Double.isFinite(v.value())) return Status.NONFINITE;
            MechanismLimit limit=policy.mechanisms().get(v.channel());
            if(!channels.add(v.channel())||limit==null||!limit.unit().equals(v.unit())||v.value()<limit.min()||v.value()>limit.max()||("bool".equals(v.unit())&&v.value()!=0&&v.value()!=1)) return Status.OUT_OF_DOMAIN;
        }
        return null;
    }
    /** Full bounded validation belongs in the predictor worker, off the periodic control path. */
    public Decision evaluate(Request r, Reply reply, World.WorldSnapshot current, long nowUs) {
        Objects.requireNonNull(r); Objects.requireNonNull(current); time(nowUs);
        if(current.tracks().size()>MAX_HISTORY_TRACKS) return reject(Status.INVALID_INPUT,"current track capacity");
        if(r.cancellation().cancelled()) return reject(Status.CANCELLED,"request cancelled");
        if(!SCHEMA_VERSION.equals(r.schemaVersion())||!policy.modelCalibrations().containsKey(r.modelId())) return reject(Status.UNSUPPORTED_SCHEMA,"request schema/model unsupported");
        if(r.frame()!=Frame.BLUE_FIELD||r.timestampDomain()!=TimestampDomain.ROBOT_MONOTONIC_US) return reject(Status.INVALID_INPUT,"unsupported input domains");
        if(r.snapshotId()>current.id()||r.epoch()!=current.epoch()||!r.field().equals(current.field())) return reject(Status.MISMATCHED_IDENTITY,"current world reset/regressed");
        if(!policy.allowedFields().contains(r.field())||!policy.allowedConfigurationRevisions().contains(r.source().configurationRevision())) return reject(Status.OUT_OF_DOMAIN,"model field/configuration domain not approved");
        if(r.issuedUs()>nowUs||r.historyCutoffUs()>r.issuedUs()||r.validUntilUs()<=r.issuedUs()||nowUs>r.validUntilUs()) return reject(Status.STALE_REQUEST,"request times invalid/expired");
        if(!r.sync().valid()||r.sync().maximumErrorUs()>policy.maxSyncErrorUs()) return reject(Status.UNSYNCHRONIZED,"input clock mapping outside limit");
        if(r.horizonUs()==0||r.stepUs()==0||r.horizonUs()>policy.maxHorizonUs()||r.horizonUs()%r.stepUs()!=0||r.horizonUs()/r.stepUs()>=MAX_SAMPLES) return reject(Status.DIMENSION_MISMATCH,"unsupported fixed time grid");
        if(nowUs-r.historyCutoffUs()>policy.maxHistoryAgeUs()||nowUs-r.historyCutoffUs()>r.horizonUs()) return reject(Status.STALE_REQUEST,"history cutoff/forecast horizon too old");
        long lastTime=-1,lastId=-1;
        for(HistorySample h:r.history()) {
            World.WorldSnapshot s=h.snapshot(); long t=s.ego().timeUs();
            if(s.epoch()!=r.epoch()||!s.field().equals(r.field())||t<=lastTime||s.id()<=lastId||t>r.historyCutoffUs()||r.historyCutoffUs()-t>policy.maxHistoryAgeUs()) return reject(Status.INVALID_INPUT,"history epoch/field/time/order");
            lastTime=t; lastId=s.id();
            if(h.validMask()&&(!s.ego().localizationValid()||!inside(s.ego().fieldPose().position().x(),s.ego().fieldPose().position().y())||s.ego().fieldVelocity().linearMps().norm()>policy.maxSpeedMps()||Math.abs(s.ego().fieldVelocity().angularRadPerSec())>policy.maxOmegaRadps())) return reject(Status.OUT_OF_DOMAIN,"valid history ego outside domain");
            Set<Long> trackIds=new HashSet<>();
            for(World.ObjectTrack t2:s.tracks()) {
                if(!trackIds.add(t2.id())||t2.estimateUs()>t||t2.lastMeasurementUs()>r.historyCutoffUs()) return reject(Status.INVALID_INPUT,"history track identity/time");
                if(t2.provenance().stream().anyMatch(stamp->stamp.captureUs()>t2.lastMeasurementUs()||stamp.publicationUs()>r.issuedUs())) return reject(Status.INVALID_INPUT,"history provenance time");
                if(h.validMask()&&(!inside(t2.positionM().x(),t2.positionM().y())||t2.velocityMps().norm()>policy.maxSpeedMps())) return reject(Status.OUT_OF_DOMAIN,"history track outside domain");
            }
            AcceptedCommand c=h.acceptedCommand();
            if(c!=null) {
                if(c.authority()!=Authority.ACCEPTED||c.frame()!=Frame.ROBOT_RELATIVE||c.issuedUs()>c.acceptedUs()||c.acceptedUs()>t) return reject(Status.INVALID_INPUT,"accepted command authority/time/frame");
                Status error=action(c.bodyTwist(),c.mechanisms()); if(error!=null) return reject(error,"accepted command outside input domain");
            }
        }
        HistorySample latest=r.history().get(r.history().size()-1);
        if(lastId!=r.snapshotId()||!latest.validMask()) return reject(Status.INVALID_INPUT,"latest valid snapshot required");
        if(current.obstacleMapVersion()!=latest.snapshot().obstacleMapVersion()) return reject(Status.MISMATCHED_IDENTITY,"current obstacle map changed");
        if(current.ego().timeUs()<latest.snapshot().ego().timeUs()||current.ego().timeUs()>nowUs) return reject(Status.INVALID_INPUT,"current ego timestamp regressed/future");
        if(nowUs-current.ego().timeUs()>policy.maxHistoryAgeUs()) return reject(Status.STALE_REQUEST,"current ego stale");
        if(!current.ego().localizationValid()||!inside(current.ego().fieldPose().position().x(),current.ego().fieldPose().position().y())||current.ego().fieldVelocity().linearMps().norm()>policy.maxSpeedMps()||Math.abs(current.ego().fieldVelocity().angularRadPerSec())>policy.maxOmegaRadps()) return reject(Status.OUT_OF_DOMAIN,"current localization outside model domain");
        Set<String> targets=new HashSet<>(),candidates=new HashSet<>(); Set<Long> trackTargets=new HashSet<>(); boolean hasEgo=false;
        for(Target target:r.targets()) {
            if(!targets.add(target.targetId())) return reject(Status.DIMENSION_MISMATCH,"duplicate target ID");
            if(target.kind()==TargetKind.EGO) { if(hasEgo) return reject(Status.DIMENSION_MISMATCH,"duplicate ego target"); hasEgo=true; }
            else if(!trackTargets.add(target.trackId())||latest.snapshot().tracks().stream().noneMatch(t->t.id()==target.trackId())) return reject(Status.INVALID_INPUT,"track target absent/duplicated");
            if(target.kind()==TargetKind.OBJECT_TRACK&&current.tracks().stream().noneMatch(t->t.id()==target.trackId())) return reject(Status.MISMATCHED_IDENTITY,"requested track no longer current");
        }
        int n=(int)(r.horizonUs()/r.stepUs()+1);
        for(ActionCandidate c:r.candidates()) {
            if(!candidates.add(c.candidateId())||c.authority()!=Authority.CANDIDATE||c.semantics()!=ActionSemantics.BODY_TWIST_OPEN_LOOP_V1||c.frame()!=Frame.ROBOT_RELATIVE||c.samples().size()!=n) return reject(Status.DIMENSION_MISMATCH,"candidate authority/frame/dimensions");
            for(int i=0;i<n;i++) { ActionSample s=c.samples().get(i); if(s.offsetUs()!=i*r.stepUs()) return reject(Status.DIMENSION_MISMATCH,"candidate time grid"); Status error=action(s.bodyTwist(),s.mechanisms()); if(error!=null) return reject(error,"candidate outside input domain"); }
        }
        if(reply==null) return reject(Status.ABSENT,"predictor absent");
        if(!SCHEMA_VERSION.equals(reply.schemaVersion())) return reject(Status.UNSUPPORTED_SCHEMA,"reply schema unsupported");
        if(!reply.modelId().equals(r.modelId())||!reply.requestId().equals(r.requestId())||reply.snapshotId()!=r.snapshotId()||reply.epoch()!=r.epoch()||!reply.field().equals(r.field())||!reply.source().equals(r.source())||reply.historyCutoffUs()!=r.historyCutoffUs()||reply.frame()!=r.frame()||reply.timestampDomain()!=r.timestampDomain()) return reject(Status.MISMATCHED_IDENTITY,"reply identity/cutoff/domain mismatch");
        if(reply.generatedUs()>nowUs) return reject(Status.FUTURE_REPLY,"reply generation in future");
        if(reply.generatedUs()<r.issuedUs()||reply.validUntilUs()<reply.generatedUs()||reply.validUntilUs()>r.validUntilUs()||nowUs>reply.validUntilUs()) return reject(Status.STALE_REPLY,"reply time window invalid/expired");
        if(reply.kind()!=ForecastKind.LEARNED_FORECAST||reply.horizonUs()!=r.horizonUs()||reply.stepUs()!=r.stepUs()||reply.forecasts().size()!=targets.size()*candidates.size()) return reject(Status.DIMENSION_MISMATCH,"forecast layout mismatch");
        if(!policy.modelCalibrations().get(reply.modelId()).contains(reply.uncertaintyCalibrationId())) return reject(Status.UNCALIBRATED,"model-specific uncertainty calibration not approved; confidence is insufficient");
        Set<Pair> pairs=new HashSet<>(); boolean anyUsable=false;
        for(Forecast f:reply.forecasts()) {
            if(!targets.contains(f.targetId())||!candidates.contains(f.candidateId())||!pairs.add(new Pair(f.candidateId(),f.targetId()))||f.samples().size()!=n) return reject(Status.DIMENSION_MISMATCH,"forecast pair/sample mismatch");
            Target target=r.targets().stream().filter(t->t.targetId().equals(f.targetId())).findFirst().orElseThrow();
            Vec2 anchor=target.kind()==TargetKind.EGO?latest.snapshot().ego().fieldPose().position():latest.snapshot().tracks().stream().filter(t->t.id()==target.trackId()).findFirst().orElseThrow().positionM();
            ForecastSample previous=null;
            for(int i=0;i<n;i++) {
                ForecastSample s=f.samples().get(i);
                if(s.offsetUs()!=i*r.stepUs()) return reject(Status.DIMENSION_MISMATCH,"forecast time grid");
                if(!finiteValues(s.xM(),s.yM(),s.vxMps(),s.vyMps(),s.covarianceXxM2(),s.covarianceXyM2(),s.covarianceYyM2())||(s.confidence()!=null&&!Double.isFinite(s.confidence()))) return reject(Status.NONFINITE,"nonfinite forecast");
                if(s.confidence()!=null&&(s.confidence()<0||s.confidence()>1)) return reject(Status.OUT_OF_DOMAIN,"confidence outside [0,1]");
                if(!s.validMask()) { if(s.xM()!=0||s.yM()!=0||s.vxMps()!=0||s.vyMps()!=0||s.covarianceXxM2()!=0||s.covarianceXyM2()!=0||s.covarianceYyM2()!=0||s.confidence()!=null) return reject(Status.INVALID_MASK,"masked values must be canonical zero/null"); continue; }
                if(s.offsetUs()>=nowUs-r.historyCutoffUs()) anyUsable=true;
                if(!inside(s.xM(),s.yM())||Math.hypot(s.vxMps(),s.vyMps())>policy.maxSpeedMps()) return reject(Status.OUT_OF_DOMAIN,"forecast outside configured field/speed domain");
                if(Math.hypot(s.xM()-anchor.x(),s.yM()-anchor.y())>policy.maxInitialOffsetM()+policy.maxSpeedMps()*(s.offsetUs()/1_000_000.0)) return reject(Status.OUT_OF_DOMAIN,"forecast displacement from input anchor");
                if(previous!=null&&Math.hypot(s.xM()-previous.xM(),s.yM()-previous.yM())>policy.maxSpeedMps()*((s.offsetUs()-previous.offsetUs())/1_000_000.0)+1e-9) return reject(Status.OUT_OF_DOMAIN,"forecast exceeds physical displacement bound");
                previous=s;
                double xx=s.covarianceXxM2(),xy=s.covarianceXyM2(),yy=s.covarianceYyM2();
                double spread=Math.hypot(xx-yy,2*xy),maxEigen=(xx+yy+spread)/2,minEigen=(xx+yy-spread)/2;
                if(xx<0||yy<0||xy*xy>xx*yy+1e-12||minEigen<policy.minVarianceM2()||maxEigen>policy.maxVarianceM2()) return reject(Status.INVALID_UNCERTAINTY,"covariance not bounded PSD");
            }
        }
        if(!anyUsable) return reject(Status.INVALID_MASK,"no remaining valid forecast samples; elapsed offsets are never rebased");
        return new Decision(Status.ACCEPTED,"validated advisory forecast",reply);
    }
}
