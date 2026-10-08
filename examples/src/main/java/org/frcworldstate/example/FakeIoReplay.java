package org.frcworldstate.example;

import java.util.*;
import org.frcworldstate.core.*;
import static org.frcworldstate.core.Geometry.*;

/** Scripted, CPU-only robot feedback. No robot project, networking, actuator, model or hardware. */
public final class FakeIoReplay {
    private static final Uncertainty U=new Uncertainty(.001,0,.001);
    private static final FieldIdentity FIELD=new FieldIdentity("OFFSEASON_2026","synthetic-20m","v1");
    private static final Pose2 START=new Pose2(new Vec2(1,1),0);
    private static final Pose2 ARRIVAL=new Pose2(new Vec2(1.5,1),0);
    private FakeIoReplay() {}
    public static void main(String[] args) {
        List<String> a=run(false),b=run(true);
        if(!a.equals(b))throw new AssertionError("missing/rejected optional predictor changed baseline");
        for(String line:a)System.out.println(line);
        System.out.println("Replay identical with absent and rejected optional prediction; success uses scripted possession feedback.");
        replayTrackerInputs();
    }
    private static List<String> run(boolean rejectedOptionalPrediction) {
        var footprint=new PlannerBackend.Footprint(.4,.4);
        var safety=new SafetySupervisor(new SafetySupervisor.Config(100_000,100_000,25_000,1,.1,.05,2,
                footprint,true,SafetySupervisor.Action.HOLD,Set.of("AUTO")));
        var motion=new PickupController.Motion(new Vec2(.5,0),.05,.1,.05,.1,20_000,30_000,.05,.1,.01);
        var planning=new PickupController.Planning(footprint,new PlannerBackend.Constraints(2,2,2),
                new PlannerBackend.Bounds(0,0,20,20),100_000_000,.15,.1);
        var config=new PickupController.Config("piece",.7,500_000,100_000,2_000_000,50_000,.2,
                5_000_000,200_000,150_000,150_000,20_000,2,1,.3,motion,planning);
        var controller=new PickupController(config,safety,PickupController.distanceScorer((target,s)->
                target.positionM().x()>footprint.boundingRadiusM()&&target.positionM().x()<20-footprint.boundingRadiusM()));
        controller.start("fake-pickup-1",1,100_000);
        PlannerBackend fakeBackend=request->new PlannerBackend.Result(request.requestId(),request.taskId(),request.epoch(),
                request.snapshotId(),request.obstacleMapVersion(),PlannerBackend.Status.SUCCESS,
                new PlannerBackend.GeometricPath(List.of(request.startPose(),request.goal().pose())),null,1000,
                request.validUntilUs(),"scripted-straight-line","empty synthetic map; geometry only");
        long[] times={100_000,110_000,120_000,140_000,150_000,160_000,170_000,190_000,200_000};
        List<String> outputs=new ArrayList<>();PlannerBackend.Result reply=null;World.WorldSnapshot original=null;
        for(int i=0;i<times.length;i++) {
            long now=times[i];Pose2 pose=i<2?START:ARRIVAL;
            var ego=new World.EgoState(now,pose,Velocity2.zero(),U,true);
            World.RobotPort fakeRobot=()->new World.RobotFacts(ego,true,true,"AUTO",false,true,true,now>=160_000,true,now);
            var track=new World.ObjectTrack(1,1,"piece",new Vec2(2,1),new Vec2(0,0),now,now,.95,U,
                    World.TrackLifecycle.CONFIRMED,World.EstimateKind.MEASURED,List.of());
            var snapshot=new World.WorldSnapshot(i+1,1,1,FIELD,ego,List.of(track));
            var alignment=now>=150_000?new PickupController.Alignment(1,1,now,new Vec2(.5,0),U):null;
            // Optional predictor abstention never changes baseline robot facts or task controller inputs.
            var optional=optionalPrediction(snapshot,now,rejectedOptionalPrediction);
            if(!optional.usesBaseline()||optional.status()!=(rejectedOptionalPrediction?PredictionGate.Status.UNCALIBRATED:PredictionGate.Status.ABSENT))
                throw new AssertionError("optional gate did not select baseline");
            var decision=controller.update(snapshot,fakeRobot.read(),new PickupController.Update(now,System.nanoTime(),1000,
                    List.of(),true,alignment,reply));reply=null;
            outputs.add(decision.id()+" "+now+"us "+decision.state()+" "+decision.intent()+" complete="+decision.completionEvent());
            if(decision.plannerRequest()!=null) {
                original=snapshot;var request=decision.plannerRequest();
                reply=PlannerValidation.validate(request,fakeBackend.plan(request),original,1,1,now);
            }
        }
        if(controller.state()!=PickupController.State.SUCCEEDED)throw new AssertionError("scripted measured pickup failed");
        if(outputs.stream().filter(x->x.endsWith("complete=true")).count()!=1)throw new AssertionError("duplicate completion");
        return outputs;
    }
    private static PredictionGate.Decision optionalPrediction(World.WorldSnapshot snapshot,long now,boolean rejected) {
        var source=new Predictor.SourceContext("fake-boot","fake-localization","fake-config");
        var actions=List.of(new Predictor.ActionSample(0,new Predictor.BodyTwist(0,0,0),List.of()),
            new Predictor.ActionSample(100_000,new Predictor.BodyTwist(0,0,0),List.of()),
            new Predictor.ActionSample(200_000,new Predictor.BodyTwist(0,0,0),List.of()));
        var request=new Predictor.Request(Predictor.SCHEMA_VERSION,"fake-model","optional-"+snapshot.id(),snapshot.id(),snapshot.epoch(),FIELD,source,
            Predictor.Frame.BLUE_FIELD,Predictor.TimestampDomain.ROBOT_MONOTONIC_US,new Predictor.SyncMetadata("robot-port","direct-fake-clock",true,0),
            now,now,now+1_000_000,200_000,100_000,List.of(new Predictor.HistorySample(snapshot,true,null)),
            List.of(new Predictor.Target("ego",Predictor.TargetKind.EGO,null)),
            List.of(new Predictor.ActionCandidate("hold",Predictor.Authority.CANDIDATE,Predictor.ActionSemantics.BODY_TWIST_OPEN_LOOP_V1,Predictor.Frame.ROBOT_RELATIVE,actions)),()->false);
        var gate=new PredictionGate(new PredictionGate.Policy(Map.of("fake-model",Set.of("approved-synthetic-calibration")),Set.of(FIELD),Set.of("fake-config"),
            0,0,20,20,2,2,.0001,1,.1,1_000_000,500_000,1000,Map.of()));
        Predictor.Reply reply=null;
        if(rejected) {
            var samples=new ArrayList<Predictor.ForecastSample>();
            for(long t=0;t<=200_000;t+=100_000)samples.add(new Predictor.ForecastSample(t,true,
                snapshot.ego().fieldPose().position().x(),snapshot.ego().fieldPose().position().y(),0,0,.01,0,.01,null));
            reply=new Predictor.Reply(Predictor.SCHEMA_VERSION,"fake-model",request.requestId(),snapshot.id(),snapshot.epoch(),FIELD,source,
                Predictor.Frame.BLUE_FIELD,Predictor.TimestampDomain.ROBOT_MONOTONIC_US,now,now,now+500_000,200_000,100_000,
                Predictor.ForecastKind.LEARNED_FORECAST,"",List.of(new Predictor.Forecast("hold","ego",samples)));
        }
        // In real integration this bounded full validation belongs in the model worker.
        return gate.evaluate(request,reply,snapshot,now);
    }
    private static void replayTrackerInputs() {
        var config=new WorldEngine.Config(8,64,8,500_000,700_000,20_000,30_000,.8,.5,.25,.01,4,2,
                .02,10,.05,1,.2,.01,false);
        var engine=new WorldEngine(config,new PoseHistory(20,2_000_000),FIELD);
        var e0=new World.EgoState(100_000,START,Velocity2.zero(),U,true);
        var e1=new World.EgoState(200_000,START,Velocity2.zero(),U,true);
        engine.addEgo(e0);engine.addEgo(e1);
        var stamp=new World.SourceStamp("synthetic-camera","boot1","cal1","mount1","group1",1,150_000,190_000,-.3);
        engine.accept(new World.ObservationFrame(0,stamp,List.of(new World.Observation("d1","piece",.9,new Vec2(1,0),U))),200_000);
        engine.publish(e1);
        engine.correctPoses(p->new Pose2(p.position().add(new Vec2(.1,0)),p.headingRad()),200_000);
        engine.publish(new World.EgoState(200_000,new Pose2(new Vec2(1.1,1),0),Velocity2.zero(),U,true));
        engine.reset(World.ResetReason.FIELD_CHANGED,new FieldIdentity("OFFSEASON_2026","synthetic-map-2","v2"),true,300_000);
        var replay=new WorldEngine(config,new PoseHistory(20,2_000_000),FIELD);
        var recorded=engine.decisionLog();
        for(var event:recorded) {
            var input=event.input();
            if(input instanceof WorldEngine.EgoInput e)replay.addEgo(e.ego());
            else if(input instanceof WorldEngine.FrameInput f) {
                var d=replay.accept(f.frame(),event.timeUs());
                if(d.decisionId()!=event.decisionId()||d.status()!=f.status()||d.acceptedMeasurements()!=f.acceptedMeasurements())throw new AssertionError("frame replay mismatch");
            } else if(input instanceof WorldEngine.SnapshotInput s) {
                if(!replay.publish(s.snapshot().ego()).equals(s.snapshot()))throw new AssertionError("snapshot replay mismatch");
            } else if(input instanceof WorldEngine.PoseCorrectionInput c)replay.applyCorrectedHistory(c.correctedHistory(),event.timeUs());
            else if(input instanceof WorldEngine.ResetInput r&&!r.derived())replay.reset(r.reason(),r.field(),r.clearPoseHistory(),event.timeUs());
        }
        if(!recorded.equals(replay.decisionLog()))throw new AssertionError("input/decision replay mismatch");
        System.out.println("Tracker full-input replay identical: "+recorded.size()+" events, correction + reset included.");
    }
}
