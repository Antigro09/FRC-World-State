package org.frcworldstate.core;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import static org.frcworldstate.core.Geometry.*;
import static org.frcworldstate.core.Predictor.*;
import static org.frcworldstate.core.PredictionGate.Status.*;

/** CPU-only checks and Java-emitted JSON vectors; no model/runtime/JSON library needed. */
public final class PredictionContractTest {
    private static int checks;
    private static void check(boolean ok,String message) { checks++; if(!ok) throw new AssertionError(message); }
    public static FieldIdentity field() { return new FieldIdentity("OFFSEASON_2026","synthetic-field-v1","geometry-v1"); }
    private static World.WorldSnapshot snapshot(long id,long timeUs) {
        var stamp=new World.SourceStamp("camera-front","camera-boot-1","calibration-v1","mount-v1","same-capture",id, timeUs-30_000,timeUs-25_000,0.0);
        var track=new World.ObjectTrack(100,7,"piece",new Vec2(3,2),new Vec2(0,0),timeUs,timeUs-30_000,0.9,new Uncertainty(0.04,0,0.04),World.TrackLifecycle.CONFIRMED,World.EstimateKind.PROPAGATED,List.of(stamp));
        return new World.WorldSnapshot(id,7,3,field(),new World.EgoState(timeUs,new Pose2(new Vec2(1,2),0),new Velocity2(new Vec2(0.2,0),0),new Uncertainty(0.01,0,0.01),true),List.of(track));
    }
    private static List<MechanismValue> mechanisms() { return List.of(new MechanismValue("intake_enabled","bool",1)); }
    private static HistorySample history(long id,long timeUs) {
        return new HistorySample(snapshot(id,timeUs),true,new AcceptedCommand("drive-"+id,"robot-execution",Authority.ACCEPTED,Frame.ROBOT_RELATIVE,timeUs-2_000,timeUs-1_000,new BodyTwist(0.2,0,0),mechanisms()));
    }
    public static Request fixtureRequest() {
        var samples=new ArrayList<ActionSample>(); var moving=new ArrayList<ActionSample>();
        for(long t=0;t<=200_000;t+=100_000) { samples.add(new ActionSample(t,new BodyTwist(0,0,0),mechanisms())); moving.add(new ActionSample(t,new BodyTwist(0.2,0,0),mechanisms())); }
        return new Request(SCHEMA_VERSION,"fixture-model-v1","prediction-9",42,7,field(),new SourceContext("robot-boot-1","localization-v1","robot-config-v1"),Frame.BLUE_FIELD,TimestampDomain.ROBOT_MONOTONIC_US,new SyncMetadata("robot-authority","direct-clock-v1",true,100),1_010_000,1_000_000,1_400_000,200_000,100_000,List.of(history(41,900_000),history(42,1_000_000)),List.of(new Target("ego",TargetKind.EGO,null),new Target("piece-100",TargetKind.OBJECT_TRACK,100L)),List.of(new ActionCandidate("hold",Authority.CANDIDATE,ActionSemantics.BODY_TWIST_OPEN_LOOP_V1,Frame.ROBOT_RELATIVE,samples),new ActionCandidate("transit",Authority.CANDIDATE,ActionSemantics.BODY_TWIST_OPEN_LOOP_V1,Frame.ROBOT_RELATIVE,moving)),()->false);
    }
    public static Reply fixtureReply(Request r) {
        var forecasts=new ArrayList<Forecast>();
        for(ActionCandidate candidate:r.candidates()) for(Target target:r.targets()) {
            var samples=new ArrayList<ForecastSample>();
            for(long t=0;t<=r.horizonUs();t+=r.stepUs()) {
                boolean ego=target.kind()==TargetKind.EGO,moving=candidate.candidateId().equals("transit");
                if(!ego&&!moving&&t==r.horizonUs()) samples.add(new ForecastSample(t,false,0,0,0,0,0,0,0,null));
                else samples.add(new ForecastSample(t,true,ego?1+(moving?0.2*t/1_000_000.0:0):3,2,ego&&moving?0.2:0,0,0.04,0,0.04,0.9));
            }
            forecasts.add(new Forecast(candidate.candidateId(),target.targetId(),samples));
        }
        return new Reply(r.schemaVersion(),r.modelId(),r.requestId(),r.snapshotId(),r.epoch(),r.field(),r.source(),r.frame(),r.timestampDomain(),r.historyCutoffUs(),1_020_000,1_300_000,r.horizonUs(),r.stepUs(),ForecastKind.LEARNED_FORECAST,"synthetic-calibration-v1",forecasts);
    }
    public static PredictionGate fixtureGate() {
        return new PredictionGate(new PredictionGate.Policy(Map.of("fixture-model-v1",Set.of("synthetic-calibration-v1")),Set.of(field()),Set.of("robot-config-v1"),0,0,16,8,5,10,0.0001,4,0.25,1_000_000,500_000,1_000,Map.of("intake_enabled",new PredictionGate.MechanismLimit("bool",0,1))));
    }
    private static final class R {
        String schemaVersion,modelId,requestId; long snapshotId,epoch; FieldIdentity field; SourceContext source; Frame frame; TimestampDomain timestampDomain; SyncMetadata sync;
        long issuedUs,historyCutoffUs,validUntilUs,horizonUs,stepUs; List<HistorySample> history; List<Target> targets; List<ActionCandidate> candidates; Cancellation cancellation;
        R(Request r) { schemaVersion=r.schemaVersion();modelId=r.modelId();requestId=r.requestId();snapshotId=r.snapshotId();epoch=r.epoch();field=r.field();source=r.source();frame=r.frame();timestampDomain=r.timestampDomain();sync=r.sync();issuedUs=r.issuedUs();historyCutoffUs=r.historyCutoffUs();validUntilUs=r.validUntilUs();horizonUs=r.horizonUs();stepUs=r.stepUs();history=r.history();targets=r.targets();candidates=r.candidates();cancellation=r.cancellation(); }
        Request make() { return new Request(schemaVersion,modelId,requestId,snapshotId,epoch,field,source,frame,timestampDomain,sync,issuedUs,historyCutoffUs,validUntilUs,horizonUs,stepUs,history,targets,candidates,cancellation); }
    }
    private static final class P {
        String schemaVersion,modelId,requestId,calibration; long snapshotId,epoch,cutoff,generated,expires,horizon,step; FieldIdentity field; SourceContext source; Frame frame; TimestampDomain domain; List<Forecast> forecasts;
        P(Reply p) { schemaVersion=p.schemaVersion();modelId=p.modelId();requestId=p.requestId();snapshotId=p.snapshotId();epoch=p.epoch();field=p.field();source=p.source();frame=p.frame();domain=p.timestampDomain();cutoff=p.historyCutoffUs();generated=p.generatedUs();expires=p.validUntilUs();horizon=p.horizonUs();step=p.stepUs();calibration=p.uncertaintyCalibrationId();forecasts=p.forecasts(); }
        Reply make() { return new Reply(schemaVersion,modelId,requestId,snapshotId,epoch,field,source,frame,domain,cutoff,generated,expires,horizon,step,ForecastKind.LEARNED_FORECAST,calibration,forecasts); }
    }
    private static Request change(Request r,Consumer<R> edit) { R x=new R(r);edit.accept(x);return x.make(); }
    private static Reply change(Reply p,Consumer<P> edit) { P x=new P(p);edit.accept(x);return x.make(); }
    private static Reply sample(Reply p,ForecastSample sample) {
        var forecasts=new ArrayList<>(p.forecasts()); var f=forecasts.get(0);var samples=new ArrayList<>(f.samples());samples.set(0,sample);forecasts.set(0,new Forecast(f.candidateId(),f.targetId(),samples));return change(p,x->x.forecasts=forecasts);
    }
    private static void expect(PredictionGate.Status expected,Request r,Reply p,World.WorldSnapshot current,long now) {
        var decision=fixtureGate().evaluate(r,p,current,now);check(decision.status()==expected,"expected "+expected+" got "+decision);
        check(decision.usesBaseline()==(expected!=ACCEPTED),"fallback selection");
    }
    public static void main(String[] args) throws Exception {
        Request r=fixtureRequest(); Reply p=fixtureReply(r);var current=r.history().get(1).snapshot();long now=1_030_000;
        if(args.length==2&&args[0].equals("--write-fixtures")) { Path out=Path.of(args[1]);Files.createDirectories(out);Files.writeString(out.resolve("request-v1.json"),requestJson(r));Files.writeString(out.resolve("reply-v1.json"),replyJson(p));System.out.println("Java prediction vectors emitted to "+out);return; }
        expect(ACCEPTED,r,p,current,now); expect(ABSENT,r,null,current,now);
        expect(UNSUPPORTED_SCHEMA,r,change(p,x->x.schemaVersion="unsupported/9"),current,now);
        expect(MISMATCHED_IDENTITY,r,change(p,x->x.requestId="old-request"),current,now);
        expect(MISMATCHED_IDENTITY,change(r,x->x.requestId="replacement-request"),p,current,now);
        expect(MISMATCHED_IDENTITY,r,change(p,x->x.epoch++),current,now);
        expect(MISMATCHED_IDENTITY,r,change(p,x->x.source=new SourceContext("old-boot","localization-v1","robot-config-v1")),current,now);
        expect(MISMATCHED_IDENTITY,r,change(p,x->x.cutoff--),current,now);
        expect(ACCEPTED,r,p,snapshot(43,1_020_000),now);
        expect(MISMATCHED_IDENTITY,r,p,snapshot(41,990_000),now);
        expect(MISMATCHED_IDENTITY,r,p,new World.WorldSnapshot(43,7,4,field(),snapshot(43,1_020_000).ego(),current.tracks()),now);
        expect(MISMATCHED_IDENTITY,r,p,new World.WorldSnapshot(43,8,3,field(),snapshot(43,1_020_000).ego(),List.of()),now);
        expect(MISMATCHED_IDENTITY,r,p,new World.WorldSnapshot(43,7,3,field(),snapshot(43,1_020_000).ego(),List.of()),now);
        var ego=current.ego();
        expect(OUT_OF_DOMAIN,r,p,new World.WorldSnapshot(43,7,3,field(),new World.EgoState(1_020_000,ego.fieldPose(),ego.fieldVelocity(),ego.uncertainty(),false),current.tracks()),now);
        expect(INVALID_INPUT,r,p,snapshot(43,now+1),now);
        expect(STALE_REQUEST,r,p,current,1_200_001); // Forecast offsets remain anchored to cutoff.
        expect(STALE_REQUEST,r,p,current,1_400_001);
        expect(STALE_REQUEST,change(r,x->x.historyCutoffUs=x.issuedUs+1),p,current,now);
        expect(FUTURE_REPLY,r,change(p,x->x.generated=now+1),current,now);
        expect(FUTURE_REPLY,r,change(p,x->x.generated=1_020_000_000L),current,now); // ns supplied as us
        expect(STALE_REPLY,r,change(p,x->x.generated=r.issuedUs()-1),current,now);
        expect(STALE_REPLY,r,change(p,x->x.expires=now-1),current,now);
        expect(STALE_REPLY,r,change(p,x->x.expires=r.validUntilUs()+1),current,now);
        expect(CANCELLED,change(r,x->x.cancellation=()->true),p,current,now);
        expect(UNSYNCHRONIZED,change(r,x->x.sync=new SyncMetadata("nt-adapter","map-v1",false,100)),p,current,now);
        expect(UNSYNCHRONIZED,change(r,x->x.sync=new SyncMetadata("nt-adapter","map-v1",true,1_001)),p,current,now);
        expect(UNCALIBRATED,r,change(p,x->x.calibration=""),current,now);
        expect(UNCALIBRATED,r,change(p,x->x.calibration="confidence-only"),current,now);
        expect(DIMENSION_MISMATCH,r,change(p,x->x.step++),current,now);
        expect(DIMENSION_MISMATCH,r,change(p,x->x.forecasts=List.of(p.forecasts().get(0))),current,now);
        expect(DIMENSION_MISMATCH,r,change(p,x->x.forecasts=List.of(p.forecasts().get(0),p.forecasts().get(0),p.forecasts().get(2),p.forecasts().get(3))),current,now);
        expect(DIMENSION_MISMATCH,r,sample(p,new ForecastSample(1,true,1,2,0,0,.04,0,.04,.9)),current,now);
        expect(NONFINITE,r,sample(p,new ForecastSample(0,true,Double.NaN,2,0,0,.04,0,.04,.9)),current,now);
        expect(NONFINITE,r,sample(p,new ForecastSample(0,true,1,2,0,0,.04,0,.04,Double.POSITIVE_INFINITY)),current,now);
        expect(INVALID_UNCERTAINTY,r,sample(p,new ForecastSample(0,true,1,2,0,0,0,0,0,.99)),current,now);
        expect(INVALID_UNCERTAINTY,r,sample(p,new ForecastSample(0,true,1,2,0,0,.04,.5,.04,.9)),current,now);
        expect(INVALID_UNCERTAINTY,r,sample(p,new ForecastSample(0,true,1,2,0,0,.04,.04,.04,.9)),current,now);
        expect(OUT_OF_DOMAIN,r,sample(p,new ForecastSample(0,true,17,2,0,0,.04,0,.04,.9)),current,now);
        expect(OUT_OF_DOMAIN,r,sample(p,new ForecastSample(0,true,1,2,6,0,.04,0,.04,.9)),current,now);
        expect(OUT_OF_DOMAIN,r,sample(p,new ForecastSample(0,true,3,2,0,0,.04,0,.04,.9)),current,now);
        expect(OUT_OF_DOMAIN,change(r,x->x.source=new SourceContext("robot-boot-1","localization-v1","unknown-configuration")),p,current,now);
        expect(INVALID_MASK,r,sample(p,new ForecastSample(0,false,1,2,0,0,0,0,0,null)),current,now);
        expect(INVALID_MASK,r,change(p,x->x.forecasts=p.forecasts().stream().map(f->new Forecast(f.candidateId(),f.targetId(),f.samples().stream().map(s->new ForecastSample(s.offsetUs(),false,0,0,0,0,0,0,0,null)).toList())).toList()),current,now);
        expect(OUT_OF_DOMAIN,r,change(p,x->{var fs=new ArrayList<>(p.forecasts());var f=fs.get(0);var samples=new ArrayList<>(f.samples());samples.set(1,new ForecastSample(100_000,true,2,2,0,0,.04,0,.04,.9));fs.set(0,new Forecast(f.candidateId(),f.targetId(),samples));x.forecasts=fs;}),current,now);
        expect(INVALID_INPUT,change(r,x->x.history=List.of(x.history.get(1),x.history.get(0))),p,current,now);
        expect(INVALID_INPUT,change(r,x->x.history=List.of(x.history.get(0),x.history.get(0))),p,current,now);
        expect(INVALID_INPUT,change(r,x->x.targets=List.of(new Target("missing",TargetKind.OBJECT_TRACK,999L))),p,current,now);
        expect(DIMENSION_MISMATCH,change(r,x->x.targets=List.of(x.targets.get(0),x.targets.get(0))),p,current,now);
        expect(DIMENSION_MISMATCH,change(r,x->x.stepUs=75_000),p,current,now);
        expect(INVALID_INPUT,change(r,x->{var h=x.history.get(1);x.history=List.of(x.history.get(0),new HistorySample(h.snapshot(),false,h.acceptedCommand()));}),p,current,now);
        expect(INVALID_INPUT,change(r,x->{var h=x.history.get(1);var c=h.acceptedCommand();var bad=new AcceptedCommand(c.commandId(),c.sourceId(),Authority.CANDIDATE,c.frame(),c.issuedUs(),c.acceptedUs(),c.bodyTwist(),c.mechanisms());x.history=List.of(x.history.get(0),new HistorySample(h.snapshot(),true,bad));}),p,current,now);
        expect(OUT_OF_DOMAIN,change(r,x->{var c=x.candidates.get(0);var actions=new ArrayList<>(c.samples());actions.set(0,new ActionSample(0,new BodyTwist(0,0,0),List.of(new MechanismValue("intake_enabled","ratio",1))));x.candidates=List.of(new ActionCandidate(c.candidateId(),c.authority(),c.semantics(),c.frame(),actions),x.candidates.get(1));}),p,current,now);
        // A missing model reply is only a baseline choice; robot-owned facts and target selection stay unchanged.
        long deterministicTarget=current.tracks().get(0).id();var absent=fixtureGate().evaluate(r,null,current,now);
        check(absent.usesBaseline()&&deterministicTarget==100&&current.equals(r.history().get(1).snapshot()),"baseline immutable with missing predictor");
        try { new Target("huge",TargetKind.OBJECT_TRACK,MAX_WIRE_INTEGER+1);throw new AssertionError("huge wire ID accepted"); } catch(IllegalArgumentException expected) { check(true,"exact wire integer bound"); }
        try { new Target("x".repeat(257),TargetKind.EGO,null);throw new AssertionError("long wire ID accepted"); } catch(IllegalArgumentException expected) { check(true,"wire identity length bound"); }
        Path fixtures=Path.of(args.length==1?args[0]:"fixtures/prediction");
        check(Files.readString(fixtures.resolve("request-v1.json")).equals(requestJson(r)),"Java request golden vector");
        check(Files.readString(fixtures.resolve("reply-v1.json")).equals(replyJson(p)),"Java reply golden vector");
        System.out.println("PredictionContractTest passed "+checks+" assertions");
    }
    private static Map<String,Object> map(Object... pairs) { var result=new LinkedHashMap<String,Object>();for(int i=0;i<pairs.length;i+=2) result.put((String)pairs[i],pairs[i+1]);return result; }
    private static Map<String,Object> fieldMap(FieldIdentity f) { return map("season",f.season(),"map_id",f.mapId(),"geometry_revision",f.geometryRevision()); }
    private static Map<String,Object> sourceMap(SourceContext s) { return map("robot_boot_id",s.robotBootId(),"localization_revision",s.localizationRevision(),"configuration_revision",s.configurationRevision()); }
    private static Map<String,Object> vec(Vec2 v) { return map("x",v.x(),"y",v.y()); }
    private static Map<String,Object> covariance(Uncertainty c) { return map("xx_m2",c.xxM2(),"xy_m2",c.xyM2(),"yy_m2",c.yyM2()); }
    private static Map<String,Object> twist(BodyTwist b) { return map("vx_mps",b.vxMps(),"vy_mps",b.vyMps(),"omega_radps",b.omegaRadps()); }
    private static List<?> mechanismMaps(List<MechanismValue> list) { return list.stream().map(m->map("channel",m.channel(),"unit",m.unit(),"value",m.value())).toList(); }
    private static Object command(AcceptedCommand c) { return c==null?null:map("command_id",c.commandId(),"source_id",c.sourceId(),"authority",c.authority().name(),"frame",c.frame().name(),"issued_us",c.issuedUs(),"accepted_us",c.acceptedUs(),"body_twist",twist(c.bodyTwist()),"mechanisms",mechanismMaps(c.mechanisms())); }
    private static Map<String,Object> snapshotMap(World.WorldSnapshot s) {
        var e=s.ego();var ego=map("time_us",e.timeUs(),"field_pose",map("position_m",vec(e.fieldPose().position()),"heading_rad",e.fieldPose().headingRad()),"field_velocity",map("linear_mps",vec(e.fieldVelocity().linearMps()),"angular_radps",e.fieldVelocity().angularRadPerSec()),"uncertainty",covariance(e.uncertainty()),"localization_valid",e.localizationValid());
        var tracks=s.tracks().stream().map(t->map("id",t.id(),"epoch",t.epoch(),"object_class",t.objectClass(),"position_m",vec(t.positionM()),"velocity_mps",vec(t.velocityMps()),"estimate_us",t.estimateUs(),"last_measurement_us",t.lastMeasurementUs(),"confidence",t.confidence(),"uncertainty",covariance(t.uncertainty()),"lifecycle",t.lifecycle().name(),"estimate_kind",t.kind().name(),"provenance",t.provenance().stream().map(p->map("source_id",p.sourceId(),"boot_id",p.bootId(),"calibration_revision",p.calibrationRevision(),"mount_revision",p.mountRevision(),"correlation_group",p.correlationGroup(),"sequence",p.sequence(),"capture_us",p.captureUs(),"publication_us",p.publicationUs(),"target_height_m",p.targetHeightM())).toList())).toList();
        return map("id",s.id(),"epoch",s.epoch(),"obstacle_map_version",s.obstacleMapVersion(),"field",fieldMap(s.field()),"ego",ego,"tracks",tracks);
    }
    public static String requestJson(Request r) {
        return json(map("schema_version",r.schemaVersion(),"model_id",r.modelId(),"request_id",r.requestId(),"snapshot_id",r.snapshotId(),"epoch",r.epoch(),"field",fieldMap(r.field()),"source",sourceMap(r.source()),"frame",r.frame().name(),"timestamp_domain",r.timestampDomain().name(),"history_cutoff_us",r.historyCutoffUs(),"valid_until_us",r.validUntilUs(),"horizon_us",r.horizonUs(),"step_us",r.stepUs(),"sync",map("source_id",r.sync().sourceId(),"mapping_revision",r.sync().mappingRevision(),"valid",r.sync().valid(),"maximum_error_us",r.sync().maximumErrorUs()),"issued_us",r.issuedUs(),"history",r.history().stream().map(h->map("snapshot",snapshotMap(h.snapshot()),"valid_mask",h.validMask(),"accepted_command",command(h.acceptedCommand()))).toList(),"targets",r.targets().stream().map(t->map("target_id",t.targetId(),"kind",t.kind().name(),"track_id",t.trackId())).toList(),"candidates",r.candidates().stream().map(c->map("candidate_id",c.candidateId(),"authority",c.authority().name(),"semantics",c.semantics().name(),"frame",c.frame().name(),"samples",c.samples().stream().map(a->map("offset_us",a.offsetUs(),"body_twist",twist(a.bodyTwist()),"mechanisms",mechanismMaps(a.mechanisms()))).toList())).toList()))+"\n";
    }
    public static String replyJson(Reply p) {
        return json(map("schema_version",p.schemaVersion(),"model_id",p.modelId(),"request_id",p.requestId(),"snapshot_id",p.snapshotId(),"epoch",p.epoch(),"field",fieldMap(p.field()),"source",sourceMap(p.source()),"frame",p.frame().name(),"timestamp_domain",p.timestampDomain().name(),"history_cutoff_us",p.historyCutoffUs(),"valid_until_us",p.validUntilUs(),"horizon_us",p.horizonUs(),"step_us",p.stepUs(),"generated_us",p.generatedUs(),"kind",p.kind().name(),"uncertainty_calibration_id",p.uncertaintyCalibrationId(),"forecasts",p.forecasts().stream().map(f->map("candidate_id",f.candidateId(),"target_id",f.targetId(),"samples",f.samples().stream().map(s->map("offset_us",s.offsetUs(),"valid_mask",s.validMask(),"x_m",s.xM(),"y_m",s.yM(),"vx_mps",s.vxMps(),"vy_mps",s.vyMps(),"covariance_xx_m2",s.covarianceXxM2(),"covariance_xy_m2",s.covarianceXyM2(),"covariance_yy_m2",s.covarianceYyM2(),"confidence",s.confidence())).toList())).toList()))+"\n";
    }
    private static String json(Object value) {
        if(value==null)return "null";
        if(value instanceof String s) { var out=new StringBuilder("\"");for(int i=0;i<s.length();i++){char c=s.charAt(i);switch(c){case '"'->out.append("\\\"");case '\\'->out.append("\\\\");case '\n'->out.append("\\n");case '\r'->out.append("\\r");case '\t'->out.append("\\t");default->{if(c<32)out.append(String.format("\\u%04x",(int)c));else out.append(c);}}}return out.append('"').toString(); }
        if(value instanceof Number n) {if(!Double.isFinite(n.doubleValue()))throw new IllegalArgumentException("JSON requires finite numbers");return value.toString();}
        if(value instanceof Boolean)return value.toString();
        if(value instanceof List<?> list)return "["+String.join(",",list.stream().map(PredictionContractTest::json).toList())+"]";
        if(value instanceof Map<?,?> map)return "{"+String.join(",",map.entrySet().stream().map(e->json(e.getKey())+":"+json(e.getValue())).toList())+"}";
        throw new IllegalArgumentException("unsupported JSON value");
    }
}
