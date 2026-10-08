package org.frcworldstate.core;

import java.util.*;
import static org.frcworldstate.core.Geometry.*;

public final class EnvelopeTest {
    private static int checks;
    private static void check(boolean ok,String msg){checks++;if(!ok)throw new AssertionError(msg);}
    public static void run() {
        var config=new ObstacleEnvelopeBuilder.Config(Map.of("piece",new ObstacleEnvelopeBuilder.ClassGeometry(.15,2,true)),2,500_000,100_000,1_000_000,1000,.05);
        var builder=new ObstacleEnvelopeBuilder(config);
        var field=new FieldIdentity("offseason","test","g1");
        var ego=new World.EgoState(1_000_000,new Pose2(new Vec2(1,1),0),Velocity2.zero(),new Uncertainty(.01,0,.01),true);
        var track=new World.ObjectTrack(1,0,"piece",new Vec2(3,2),new Vec2(2,0),1_000_000,500_000,.9,new Uncertainty(.04,0,.04),World.TrackLifecycle.COASTING,World.EstimateKind.PROPAGATED,List.of());
        var snapshot=new World.WorldSnapshot(1,0,1,field,ego,List.of(track));
        var decision=builder.build(snapshot,1_000_000,1_200_000,true);check(decision.usable(),"fresh bounded geometry builds");
        var obstacle=decision.obstacles().get(0);check(obstacle.centerM().equals(new Vec2(2,2)),"envelope anchors actual last measured center, not freshened extrapolation");
        check(Math.abs(obstacle.radiusM()-.15)<1e-10,"physical dimension preserved");
        check(Math.abs(obstacle.uncertaintyMarginM()-(.4+2*.751))<1e-10,"covariance+age+motion+timing+latency derived");
        check(obstacle.validUntilUs()==1_200_000,"whole planning interval validity");
        check(obstacle.centerM().add(new Vec2(2*.7,0)).subtract(obstacle.centerM()).norm()<obstacle.envelopeRadiusM(),"possible future dynamic motion covered");
        check(builder.build(snapshot,1_100_001,1_200_000,true).status()==ObstacleEnvelopeBuilder.Status.STALE_INFORMATION,"stale last measurement never freshened by estimate timestamp");
        var empty=new World.WorldSnapshot(2,0,1,field,ego,List.of());
        check(!builder.build(empty,1_000_000,1_200_000,false).usable(),"empty track list cannot establish free coverage");
        var unknown=new ObstacleEnvelopeBuilder(new ObstacleEnvelopeBuilder.Config(Map.of(),2,500_000,100_000,1_000_000,1000,.05));
        check(unknown.build(snapshot,1_000_000,1_200_000,true).status()==ObstacleEnvelopeBuilder.Status.UNKNOWN_GEOMETRY,"missing dimensions rejected");
        check(builder.build(snapshot,1_000_000,2_000_001,true).status()==ObstacleEnvelopeBuilder.Status.INVALID_INPUT,"unbounded envelope horizon rejected");
        System.out.println("EnvelopeTest: "+checks+" checks passed");
    }
}
