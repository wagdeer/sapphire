// Reuse the accepted B2 fixture/oracle and its synchronized fault hook discipline.
#define main b2_fixture_main
#include "continuation_test.cpp"
#undef main

bool capture(PoseGraphBackend &b, bool navigation, bool try_only, std::uint64_t &r,
             std::uint64_t &s, std::shared_ptr<const NavigationGrid> &grid) {
  std::string u; std::uint64_t g; double t; Eigen::Isometry3d c,p;
  bool ready=b.captureReady(navigation,try_only,u,r,g,s,t,c,p,grid);
  if(ready) {
    check(u==b.mapUuid(),"captured UUID");
    check(t==double(s+1),"original source anchor timestamp");
    if(navigation) check(grid&&grid->map_uuid==u&&grid->source_graph_revision==r,"grid source attribution");
    check(p.matrix().allFinite()&&c.matrix().allFinite(),"coherent pose/correction");
  }
  return ready;
}
std::string persisted(const fs::path &path) {
  MapDatabase db(path.string(), "resume"); auto handle=MapDatabaseTestAccess::handle(db);
  std::ostringstream out;
  for (auto query : {"SELECT name,sql FROM sqlite_master ORDER BY name", "SELECT * FROM MapState", "SELECT * FROM Node ORDER BY id", "SELECT * FROM Link ORDER BY from_id,to_id", "PRAGMA user_version"}) {
    database_detail::Statement statement(handle,query);
    while(sqlite3_step(statement.get())==SQLITE_ROW) {
      for(int col=0;col<sqlite3_column_count(statement.get());++col) {
        int type=sqlite3_column_type(statement.get(),col),size=sqlite3_column_bytes(statement.get(),col);
        out<<type<<':'<<size<<':'; auto data=sqlite3_column_blob(statement.get(),col);
        if(size)out.write(static_cast<const char*>(data),size);
      }
      out<<';';
    }
  }
  db.finish();return out.str();
}
void b3Continue(const fs::path &path) {
  PoseGraphBackend b(resumeConfig(),{},path.string(),{});
  check(!b.hasActiveCorrection(),"fresh process requires B1");
  attach(b,6); clearEvents();
  const auto root=b.continuationProgress();
  check(root.root_source==40&&root.root_revision==root.ready_revision&&root.completed_revision==root.ready_revision,"B1 source root pair");
  Barrier committed, sink;
  setHook([&](const char *p,void*){if(std::string(p)=="b2-postcommit-materialization")committed.arrive();});
  std::uint64_t pc=0,pg=0,r=0,s=0; std::shared_ptr<const NavigationGrid> old;
  check(capture(b,true,false,r,s,old),"root capture");
  auto publication=std::async(std::launch::async,[&]{sink.arrive(); pc=r;pg=r;}); sink.wait();
  submit(b,1); committed.wait();
  auto p=b.continuationProgress();
  check(p.committed_revision==root.ready_revision+1&&p.ready_revision==root.ready_revision&&pc==0&&pg==0,"C advances while Y/Pc/Pg lag");
  check(p.head_sequence==41&&p.committed_outcome_known,"observable current head and known W result");
  // No separate accessor or revision recheck may observe partially mutated owners.
  std::string u; std::uint64_t g,z;double t;Eigen::Isometry3d c,x;std::shared_ptr<const NavigationGrid> dummy;
  check(!b.captureReady(false,true,u,z,g,z,t,c,x,dummy),"busy coherent capture does not inspect mutation");
  committed.release(); awaitEvent("b2-completed"); setHook({});
  submit(b,2); awaitEvent("b2-completed",2);
  p=b.continuationProgress();
  check(p.completed_revision==root.ready_revision+2&&p.last_completed==42&&pc==0,"mapping progresses with blocked sink");
  check(old->source_graph_revision==root.ready_revision,"old immutable grid never relabeled");
  sink.release();publication.get();
  b.drain();p=b.continuationProgress();
  check(p.accepted==2&&p.completed==2&&!p.canceled_first&&!p.waiting&&!p.head_bytes,"healthy accepted drain");
  check(capture(b,true,false,r,s,dummy),"readable final interval without new input");pc=r;pg=r;
  check(pc==p.ready_revision&&pg==p.committed_revision&&s==42,"F exact group completion and direct S/R");
  std::string expected=semantic(b,9);
  std::ofstream out(path.string()+".oracle",std::ios::binary);out.write(expected.data(),expected.size());out.close();
  const auto uuid=b.mapUuid();
  b.close();
  std::ofstream(path.string()+".uuid")<<uuid;
  const auto rows=persisted(path);std::ofstream raw(path.string()+".rows",std::ios::binary);raw.write(rows.data(),rows.size());raw.close();
  bool fenced=false;try{capture(b,false,false,r,s,dummy);}catch(...){fenced=true;}check(fenced,"checked close fences reads");
  std::cout<<"B3 PROCESS_B F="<<r<<" S="<<s<<" Pc="<<pc<<" Pg="<<pg<<"\n";
}
void runChild(const char *mode,const fs::path &path) {
  auto pid=fork();check(pid>=0,"fork");if(!pid){execl("/proc/self/exe","publication_test",mode,path.c_str(),nullptr);_exit(120);}
  int status;check(waitpid(pid,&status,0)==pid&&status==0,std::string("process ")+mode);
}
void densePreflight() {
  NaviMapParameters config;config.resolution=.1;
  OccupancyGrid occupancy(config);LocalGrid local(.1f);local.obstacleCells={{0,0},{500,0}};
  check(occupancy.append(GridFrame(1,Eigen::Isometry3f::Identity(),local)),"sparse large extent is valid authority");
  bool rejected=false;try{occupancy.getMap();}catch(const std::length_error&){rejected=true;}
  check(rejected,"dense extent rejected before allocation");
  OccupancyGrid wide(config);LocalGrid extreme(.1f);extreme.obstacleCells={{-150000000.f,0},{150000000.f,0}};
  check(wide.append(GridFrame(1,Eigen::Isometry3f::Identity(),extreme)),"sparse signed-coordinate extremes");
  rejected=false;try{wide.getMap();}catch(const std::length_error&){rejected=true;}
  check(rejected,"subtraction widened before signed overflow; no dense allocation");
}
void legacyFailure(const fs::path &path,const std::string &fault) {
  PoseGraphParameters config;config.visual.enabled=true;NaviMapParameters nav;
  PoseGraphBackend b(config,nav,path.string(),{});Barrier barrier;
  setHook([&](const char *point,void*){if(point==fault){barrier.arrive();throw std::runtime_error("owned visual failure");}});
  b.addFrame(frame(0,anchor(0)));barrier.wait();
  std::string u;std::uint64_t r,g,s;double t;Eigen::Isometry3d c,p;std::shared_ptr<const NavigationGrid> grid;
  check(!b.captureReady(false,true,u,r,g,s,t,c,p,grid),"legacy lifecycle acquired before visual/storage mutation");
  barrier.release();bool failed=false;try{b.drain();}catch(...){failed=true;}setHook({});
  check(failed&&b.failed()&&!b.continuationProgress().ready_available,"visual failure retains backend fence and no Y");
  b.close();
}
void optionalCopies() {
  SapphireParameters config; config.pose_graph.enabled=false;config.general.save_map=0;
  int callbacks=0,drops=0,copy_entries=0;
  OutputSink output;output.trajectory=[&](auto){++callbacks;};output.optional_drop=[&](bool){++drops;};
  SlamPipeline p(config,output);p.shutdown();
  setHook([&](const char *stage,void*){if(std::string(stage)=="b3-trajectory-copy"){++copy_entries;throw std::bad_alloc();}});
  SlamPipelineTestAccess::optionalCopies(p,1);
  check(callbacks==0&&drops==1&&copy_entries==1&&!p.failed(),"output-only copy failure before callback is contained");
  SlamPipelineTestAccess::optionalCopies(p,300000);
  check(callbacks==0&&drops==2&&copy_entries==1&&!p.failed(),"oversize rejected before first trajectory copy");
  SlamPipelineTestAccess::optionalMap(p);
  check(callbacks==0&&drops==3&&copy_entries==1&&!p.failed(),"oversize rejected before second trajectory copy");
  setHook({});p.drain();p.close();
}
void invalidTail(const fs::path &path) {
  SapphireParameters config;config.pose_graph.database_path=path.string();config.general.save_map=0;
  SlamPipeline p(config);
  std::vector<LidarPoint> scan(1);scan[0].time_offset=.01;
  check(p.push_lidar(1,std::move(scan)),"raw scan accepted pending IMU");
  auto progress=p.drain();check(progress.accepted==0&&!progress.completed_revision,"missing IMU tail produces no fabricated node");p.close();
  SubmapFrameBuffer builder(4,1000,20);auto visual=frame(0,anchor(0));
  builder.push_visual(std::move(visual.visual_frames().front()),4);
  check(!builder.flush(),"image-only tail cannot fabricate geometry");
}
void stoppedHandoff(const fs::path &path) {
  seed(path,false);PoseGraphBackend b(resumeConfig(),{},path.string(),{});Barrier ready;
  setHook([&](const char *stage,void*){if(std::string(stage)=="b1-runtime-ready")ready.arrive();});
  auto attached=std::async(std::launch::async,[&]{
    auto result=b.attachFreshSession(frame(40,freshAnchor()),grid(),6,relation(),77);
    b.beginContinuation(77);return result;
  });
  ready.wait();b.stopAdmission();ready.release();auto result=attached.get();setHook({});
  check(result.status==AttachmentStatus::Attached&&!b.failed(),"stop racing B1 handoff preserves actual committed success");
  b.drain();std::uint64_t r,s;std::shared_ptr<const NavigationGrid> map;
  check(capture(b,true,false,r,s,map)&&r==result.committed_revision,"stopped handoff exposes final root target");b.close();
}
void mappingMeasurement(const fs::path &path,bool exports) {
  seed(path,false);PoseGraphBackend b(resumeConfig(),{},path.string(),{});attach(b,6);clearEvents();
  auto start=std::chrono::steady_clock::now();double export_ms=0;
  for(int i=1;i<=2;++i) {
    submit(b,i);awaitEvent("b2-completed",i);
    if(exports) {
      auto selected=std::chrono::steady_clock::now();std::uint64_t r,s;std::shared_ptr<const NavigationGrid> map;
      check(capture(b,true,false,r,s,map),"measured export");
      export_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-selected).count();
    }
  }
  auto mapped=std::chrono::steady_clock::now();b.drain();auto drained=std::chrono::steady_clock::now();b.close();
  std::cout<<"B3_MEASURE exports="<<exports<<" two_head_wall_ms="<<std::chrono::duration<double,std::milli>(mapped-start).count()
    <<" capture_grid_ms="<<export_ms<<" backend_drain_ms="<<std::chrono::duration<double,std::milli>(drained-mapped).count()
    <<" storage_close_ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-drained).count()<<'\n';
}
void lowLevelOutcomes(const fs::path &root) {
  for (const std::string mode : {"success","NotCommitted","Committed","Unknown"}) {
    auto path=root/("low-level-"+mode+".db");seed(path,false);
    PoseGraphBackend b(resumeConfig(),{},path.string(),{});b.promoteStorageToWritable();
    const auto before=b.continuationProgress();const auto previous=*before.committed_revision;
    auto q=frame(6,anchor(6));GraphLink base{6,7,0,(anchor(5).inverse()*anchor(6)).cast<float>()};
    if(mode=="Unknown") ambiguous_commit=true;
    else if(mode!="success") setHook([&](const char *point,void*) {
      if(std::string(point)==(mode=="Committed"?"after-commit":"before-commit"))throw std::runtime_error("R2 original "+mode);
    });
    std::exception_ptr cause;
    try {b.commitFinalizedSubmap(q,grid(),buildVisualScene(q)->encode(),{{7,anchor(6).cast<float>()}},base,std::nullopt,b.mapUuid(),previous,7,1);}
    catch(...) {cause=std::current_exception();}
    setHook({});const auto p=b.continuationProgress();
    const auto outcome=mode=="Unknown"?CommitOutcome::Unknown:mode=="NotCommitted"?CommitOutcome::NotCommitted:CommitOutcome::Committed;
    check(p.last_commit_outcome==outcome&&p.committed_outcome_known==(mode!="Unknown"),"R2 authoritative exceptional outcome mirror");
    check(p.committed_revision==previous+(outcome==CommitOutcome::Committed)&&p.committed_revision==b.graphRevision(),"R2 authoritative C mirror, no Unknown guess");
    check(p.ready_revision==before.ready_revision,"R2 low-level W never promotes unmaterialized Y");
    check(b.failed()==(mode!="success")&&bool(cause)==(mode!="success"),"R2 original health behavior");
    if(cause) {
      check(p.first_failure==cause,"R2 mirror retains original failure object");
      std::uint64_t r,source;std::shared_ptr<const NavigationGrid> g;bool fenced=false;
      try{capture(b,false,false,r,source,g);}catch(...){fenced=true;}check(fenced,"R2 no publication of failed/unready state");
      try{b.drain();check(false,"failed drain must throw");}catch(...){check(std::current_exception()==cause,"R2 first cause survives drain");}
    } else b.drain();
    b.close();std::cout<<"PASS R2 "<<mode<<" C="<<*p.committed_revision<<" Y="<<p.ready_revision<<" known="<<p.committed_outcome_known<<'\n';
  }
}
void crossedGeneration(const fs::path &root) {
  for (bool status : {false,true}) {
    SapphireParameters config;config.general.save_map=0;config.general.save_path=(root/(status?"crossed-status":"crossed-capture")).string()+"/";
    SlamPipeline p(config);p.shutdown();clearEvents();
    SlamPipelineTestAccess::backend(p).addFrame(frame(0,anchor(0)));awaitEvent("b3-legacy-ready");
    const auto old=p.continuationProgress();check(old.generation==0,"initial owner generation");
    std::string u;std::uint64_t r,g,source;double time;Eigen::Isometry3d c,x;std::shared_ptr<const NavigationGrid> map;
    Barrier during;std::function<void()> probe=[&]{during.arrive();};
    auto pending=std::async(std::launch::async,[&] {
      capture_allocation_probe=&probe;
      if(status) {auto value=p.continuationProgress();check(value.map_uuid==old.map_uuid&&value.generation==old.generation,"crossed status retains old owner generation");}
      else {check(p.captureReady(false,true,u,r,g,source,time,c,x,map),"capture validated before reset can finish");check(u==old.map_uuid&&g==*old.generation&&r==old.ready_revision,"crossed capture retains old content and old g");}
      capture_allocation_probe=nullptr;
    });
    during.wait();SlamPipelineTestAccess::resetDomainStores(p);during.release();pending.get();
    check(!p.captureReady(false,true,u,r,g,source,time,c,x,map),"reset before capture rejects invalid domain");
    check(p.continuationProgress().generation==old.generation,"status before replacement still describes old owner");
    SlamPipelineTestAccess::uniqueReplacementPath(p);SlamPipelineTestAccess::replace(p);clearEvents();
    SlamPipelineTestAccess::backend(p).addFrame(frame(0,anchor(0)));awaitEvent("b3-legacy-ready");
    check(p.captureReady(false,false,u,r,g,source,time,c,x,map)&&g==1&&u!=old.map_uuid,"new installed owner uses new generation");
    const auto captured_generation=g;SlamPipelineTestAccess::resetDomainStores(p);
    check(captured_generation==1&&!p.captureReady(false,true,u,r,g,source,time,c,x,map),"reset immediately after capture cannot relabel selected values");
    p.drain();p.close();std::cout<<"PASS R3 crossed "<<(status?"progress":"capture")<<" old_g=0 new_g=1, before/during/after reset\n";
  }
}
int main(int argc,char **argv) try {
  if(argc>1&&std::string(argv[1])=="--read") {
    auto result=b2_fixture_main(argc,argv);
    std::ifstream rows(std::string(argv[2])+".rows",std::ios::binary);std::string expected{std::istreambuf_iterator<char>(rows),{}};
    check(persisted(argv[2])==expected,"independent DB schema/Nodes/Links/serialized anchors and poses unchanged");
    std::ifstream identity(std::string(argv[2])+".uuid");std::string uuid;identity>>uuid;
    PoseGraphBackend reopened(resumeConfig(),{},argv[2],{});
    check(reopened.mapUuid()==uuid&&!reopened.continuationProgress().root_revision&&!reopened.continuationProgress().completed_revision,
          "UUID persists; active generation/source attribution does not");reopened.finish();return result;
  }
  if(argc>1&&std::string(argv[1])=="--seed"){seed(argv[2],false);return 0;}
  if(argc>1&&std::string(argv[1])=="--continue"){b3Continue(argv[2]);return 0;}
  densePreflight(); optionalCopies();
  char temp[]="/tmp/sapphire-b3-XXXXXX";check(mkdtemp(temp),"fixture dir");fs::path root=temp;
  lowLevelOutcomes(root);crossedGeneration(root);
  invalidTail(root/"invalid-tail.db"); stoppedHandoff(root/"handoff.db");
  mappingMeasurement(root/"timing-none.db",false);mappingMeasurement(root/"timing-export.db",true);
  runChild("--seed",root/"oracle.db");runChild("--continue",root/"oracle.db");
  oracle(root/"oracle.db",(root/"oracle.db").string()+".oracle",9);
  for(auto fault:{"b3-legacy-visual-insert","b3-legacy-visual-query","b3-legacy-visual-settle"})legacyFailure(root/(std::string(fault)+".db"),fault);
  std::cout<<"PASS B3 coherent lifecycle, commit/readiness lag, sink lag, final drain, visual fence, dense preflight, A/B/C oracle fixtures="<<root<<"\n";
  return 0;
}catch(const std::exception &e){std::cerr<<"FAIL B3 "<<e.what()<<'\n';return 1;}
