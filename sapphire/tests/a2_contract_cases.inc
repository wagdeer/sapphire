// Bounded A2 contract regression, included in historical_reconstruction_test.cpp.
// Faults enter through existing weak no-op test points; never suppress MapError.
void amendmentContract(const fs::path &root) {
  auto path = root / "12.db";
  PoseGraphParameters config; config.map_mode = "resume";
  const auto original = files(root);
  std::ostringstream logged;
  const auto old_logger = spdlog::default_logger();
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(logged);
  spdlog::set_default_logger(std::make_shared<spdlog::logger>("a2-test", sink));
  const auto reset = [&] { a2_hook = {}; logged.str(""); logged.clear(); };
  const auto warning = [&] { return logged.str().find("reconstruction reproduction discrepancy") != std::string::npos; };
  const auto fail = [&](const std::string &name, const std::function<void(const char *, void *)> &inject,
                        MapErrorCode code = MapErrorCode::ReconstructionDiscrepancy) {
    reset(); bool reached = false;
    a2_hook = [&](const char *stage, void *ctx) { inject(stage, ctx); reached = true; };
    errorIs(code, [&] { PoseGraphBackend backend(config, {}, path.string(), {}); });
    check(reached && !warning(), name + ": must hard fail without ordinary similarity warning");
    reset(); check(files(root) == original, name + ": unchanged persistent bytes");
    const int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC);
    check(fd >= 0 && flock(fd, LOCK_EX | LOCK_NB) == 0, name + ": ownership released"); close(fd);
    std::cout << "PASS A2 hard failure " << name << '\n';
  };
  try {
    // Independent construction oracle: actual canonical prior, directed Link bytes,
    // six variances, unwrapped type-0 and Huber(10, Block). Zero residual is insufficient.
    std::vector<GraphLink> links;
    { MapDatabase db(path.string(), "resume"); links = db.loadGraphLinks(); }
    bool inspected = false;
    a2_hook = [&](const char *stage, void *ctx) {
      if (std::string(stage) != "a2-canonical-graph") return;
      auto &graph = *static_cast<gtsam::NonlinearFactorGraph *>(ctx);
      check(graph.size() == links.size() + 1, "original factor cardinality");
      auto prior = std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(graph[0]);
      check(prior && prior->keys() == gtsam::KeyVector{0} && prior->dim() == 6, "sole original typed prior");
      check(prior->prior().matrix().isApprox(anchor(0).matrix(), 1e-12), "original Node-1 anchor prior");
      check(prior->noiseModel()->equals(*gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(1e-12)), 1e-15), "prior variances");
      gtsam::Vector6 v; v << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
      auto odom = gtsam::noiseModel::Diagonal::Variances(v);
      auto huber = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(10, gtsam::noiseModel::mEstimator::Base::Block), odom);
      for (size_t i = 0; i < links.size(); ++i) {
        auto f = std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(graph[i + 1]); const auto &l = links[i];
        check(f && f->dim() == 6 && f->keys() == gtsam::KeyVector{gtsam::Key(l.from_id-1), gtsam::Key(l.to_id-1)}, "typed directed factor");
        const Eigen::Matrix4f decoded = f->measured().matrix().cast<float>();
        check(std::memcmp(decoded.data(), l.transform.data(), 64) == 0, "measurement bytes");
        check(f->noiseModel()->equals(*(l.type == 0 ? gtsam::SharedNoiseModel(odom) : gtsam::SharedNoiseModel(huber)), 1e-15), "noise/Huber/Block");
      }
      auto loop = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph.back());
      auto robust = std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(loop->noiseModel());
      check(robust && robust->robust()->reweightScheme() == gtsam::noiseModel::mEstimator::Base::Block, "independent Block oracle (equals omits scheme)");
      inspected = true;
    };
    { PoseGraphBackend b(config, {}, path.string(), {}); check(inspected && !warning(), "inside-reference control"); }
    reset(); std::cout << "PASS A2 actual prior/keys/measurement bytes/noise/Huber Block construction oracle\n";

    const std::array<double ReconstructionDiagnostics::*,4> metrics{&ReconstructionDiagnostics::max_translation,
      &ReconstructionDiagnostics::max_rotation, &ReconstructionDiagnostics::max_residual_translation, &ReconstructionDiagnostics::max_residual_rotation};
    const std::array<double,4> reference{.035,.003,.026,.003};
    for (size_t i=0;i<4;++i) for (int side : {-1,0,1}) {
      reset(); bool hit=false;
      const double value=side==0?reference[i]:std::nextafter(reference[i],side<0?0.:INFINITY);
      a2_hook=[&](const char *stage,void *ctx) {if(std::string(stage)=="a2-diagnostics") {
        auto &d=*static_cast<ReconstructionDiagnostics *>(ctx);for(auto m:metrics)d.*m=0.;d.*metrics[i]=value;hit=true;
      }};
      { PoseGraphBackend b(config,{},path.string(),{});check(hit && (warning()==(side>0)),"strict diagnostic boundary");
        check(!b.hasActiveCorrection(),"boundary no correction");
        if(side>0) {auto log=logged.str();check(log.find(b.mapUuid())!=std::string::npos && log.find("revision="+std::to_string(b.graphRevision()))!=std::string::npos &&
          log.find("similarity exceedance alone does not reject opening")!=std::string::npos && log.find("nodes=12 factors=13")!=std::string::npos &&
          log.find("objective_delta=")!=std::string::npos,"complete attributed warning");
          const std::array<std::string,4> labels{" translation=", " rotation=", " residual_translation=", " residual_rotation="};
          const std::array<std::string,4> units{" m reference=0.035 m exceeded=", " rad reference=0.003 rad exceeded=",
            " m reference=0.026 m exceeded=", " rad reference=0.003 rad exceeded="};
          for(size_t j=0;j<labels.size();++j) {
            const auto metric=log.find(labels[j],log.find("reconstruction reproduction discrepancy"));
            check(metric!=std::string::npos,"warning metric label");
            const auto suffix=log.find(' ',metric+labels[j].size());
            check(suffix!=std::string::npos && log.compare(suffix,units[j].size(),units[j])==0,"warning metric/reference units");
          }
        }
      }
      std::cout<<"PASS A2 metric="<<i<<" boundary="<<side<<" value="<<std::setprecision(17)<<value<<" classification="<<(side>0?"exceeds_reference":"within_reference")<<" open=success\n";
    }
    reset();
    const auto at = [](const std::string &point, auto action) {
      return [=](const char *stage, void *ctx) { if (std::string(stage)==point) action(ctx); };
    };
    for(const std::string kind:{"same-count-wrong-key","missing-key","extra-key","wrong-Pose3-type-dimension"})
      fail(kind,at("a2-rebuilt-values",[=](void *ctx){auto &v=*static_cast<gtsam::Values *>(ctx);
        if(kind=="extra-key")v.insert(900,gtsam::Pose3());
        else {v.erase(1);if(kind=="same-count-wrong-key")v.insert(900,gtsam::Pose3());
          if(kind=="wrong-Pose3-type-dimension")v.insert(1,gtsam::Point3(0,0,0));}
      }));
    for(const std::string kind:{"factor-count","wrong-endpoint","wrong-direction","wrong-prior","wrong-noise-small-movement","wrong-Huber-k","wrong-Huber-scheme","wrong-type0-robust","wrong-factor-type"})
      fail(kind,at("a2-restored-graph",[=](void *ctx){auto &g=const_cast<gtsam::NonlinearFactorGraph &>(static_cast<gtsam::ISAM2 *>(ctx)->getFactorsUnsafe());
        if(kind=="factor-count"){g.push_back(g[0]);return;}
        if(kind=="wrong-prior"){auto f=std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(g[0]);auto m=f->prior().matrix();m(0,3)+=.000001;
          g[0]=std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(0,gtsam::Pose3(m),f->noiseModel());return;}
        const size_t i=kind.find("Huber")!=std::string::npos?g.size()-1:1;
        auto f=std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(g[i]);auto a=f->key1(),b=f->key2();auto noise=f->noiseModel();
        if(kind=="wrong-endpoint")b=9;if(kind=="wrong-direction")std::swap(a,b);
        if(kind=="wrong-noise-small-movement")noise=gtsam::noiseModel::Isotropic::Sigma(6,.1);
        if(kind=="wrong-Huber-k"||kind=="wrong-Huber-scheme"||kind=="wrong-type0-robust") {
          gtsam::Vector6 v;v<<1e-6,1e-6,1e-6,1e-4,1e-4,1e-4;
          noise=gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(kind=="wrong-Huber-k"?9:10,
            kind=="wrong-Huber-scheme"?gtsam::noiseModel::mEstimator::Base::Scalar:gtsam::noiseModel::mEstimator::Base::Block),gtsam::noiseModel::Diagonal::Variances(v));
        }
        if(kind=="wrong-factor-type")g[i]=std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(a,f->measured(),noise);
        else g[i]=std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(a,b,f->measured(),noise);
      }));
    for(double bad:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
      for(const std::string point:{"a2-rebuilt-values","a2-committed-values"})
        fail(point+" nonfinite pose",at(point,[=](void *ctx){auto &v=*static_cast<gtsam::Values *>(ctx);auto m=v.at<gtsam::Pose3>(1).matrix();m(0,3)=bad;v.update(1,gtsam::Pose3(m));}));
      for(const std::string point:{"a2-residual-before","a2-residual-after","a2-residual-restored"})
        fail(point+" nonfinite",at(point,[=](void *ctx){(*static_cast<gtsam::Vector *>(ctx))[0]=bad;}));
      for(const std::string point:{"a2-quaternion","a2-relative-quaternion"})
        fail(point+" nonfinite",at(point,[=](void *ctx){static_cast<Eigen::Quaterniond *>(ctx)->x()=bad;}));
      for(auto m:metrics) fail("nonfinite final metric",at("a2-diagnostics",[=](void *ctx){static_cast<ReconstructionDiagnostics *>(ctx)->*m=bad;}));
      for(auto m:{&ReconstructionDiagnostics::objective_committed,&ReconstructionDiagnostics::objective_rebuilt})
        fail("nonfinite objective",at("a2-objectives",[=](void *ctx){static_cast<ReconstructionDiagnostics *>(ctx)->*m=bad;}));
    }
    for(const std::string point:{"a2-residual-before","a2-residual-after","a2-residual-restored"})for(int dimension:{0,3,5,7})
      fail(point+" wrong dimension "+std::to_string(dimension),at(point,[=](void *ctx){*static_cast<gtsam::Vector *>(ctx)=gtsam::Vector::Zero(dimension);}));
    fail("finite residual subtraction overflow",[&](const char *stage,void *ctx){std::string n=stage;
      if(n=="a2-residual-before"||n=="a2-residual-restored")*static_cast<gtsam::Vector *>(ctx)=gtsam::Vector::Constant(6,-1e308);
      if(n=="a2-residual-after")*static_cast<gtsam::Vector *>(ctx)=gtsam::Vector::Constant(6,1e308);
    });
    for(int component:{0,3}) fail("finite residual norm overflow component "+std::to_string(component),at("a2-residual-after",[=](void *ctx){(*static_cast<gtsam::Vector *>(ctx))[component]=1e200;}));
    fail("finite pose norm overflow",at("a2-rebuilt-values",[](void *ctx){auto &v=*static_cast<gtsam::Values *>(ctx);auto m=v.at<gtsam::Pose3>(1).matrix();m(0,3)=1e200;v.update(1,gtsam::Pose3(m));}));
    fail("finite quaternion norm overflow",at("a2-quaternion",[](void *ctx){static_cast<Eigen::Quaterniond *>(ctx)->x()=1e200;}));
    fail("zero quaternion",at("a2-quaternion",[](void *ctx){static_cast<Eigen::Quaterniond *>(ctx)->coeffs().setZero();}));
    fail("finite objective subtraction overflow",at("a2-objectives",[](void *ctx){auto &d=*static_cast<ReconstructionDiagnostics *>(ctx);d.objective_committed=-1e308;d.objective_rebuilt=1e308;}));
    for(const std::string point:{"isam-update-entry","a2-rebuilt-values","runtime-full-rebuild"})
      fail(point+" exception",at(point,[](void *){throw std::runtime_error("injected solver/estimate/materialization failure");}));
    fail("bad_alloc -> Storage",at("a2-rebuilt-values",[](void *){throw std::bad_alloc();}),MapErrorCode::Storage);
    fail("MapError passthrough",at("a2-rebuilt-values",[](void *){throw MapError(MapErrorCode::StaleMap,"injected stale identity");}),MapErrorCode::StaleMap);
    path = root / "1.db";
    fail("wrong model at zero motion",at("a2-restored-graph",[](void *ctx){
      auto &g=const_cast<gtsam::NonlinearFactorGraph &>(static_cast<gtsam::ISAM2 *>(ctx)->getFactorsUnsafe());
      auto f=std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(g[0]);
      g[0]=std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(0,f->prior(),gtsam::noiseModel::Isotropic::Sigma(6,.1));
    }));
    reset(); check(files(root)==original,"all diagnostic/fault opens preserve bytes");
    // Real final read-identity failure after numerical checks, not just error passthrough.
    path = root / "stale-a2.db";fs::copy_file(root / "12.db",path);
    const auto displaced=root/"stale-a2-original.db"; bool renamed=false;
    a2_hook=at("runtime-full-rebuild",[&](void *){fs::rename(path,displaced);fs::copy_file(root/"12.db",path);renamed=true;});
    errorIs(MapErrorCode::StaleMap,[&]{PoseGraphBackend b(config,{},path.string(),{});});
    check(renamed && !warning() && bytes(displaced)==bytes(root/"12.db"),"actual stale inode fails without warning or write");
    reset();std::cout<<"PASS A2 actual stale inode during materialization/final identity\n";
    // Invalid persisted matrices must still fail before solver diagnostics.
    for(float bad:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
      path=root/(std::isnan(bad)?"nan-persisted.db":"inf-persisted.db");fs::copy_file(root/"12.db",path);
      sqlite3 *db=nullptr;check(sqlite3_open(path.c_str(),&db)==SQLITE_OK,"open invalid persisted pose fixture");
      Eigen::Matrix4f m=Eigen::Matrix4f::Identity();m(0,3)=bad;
      {database_detail::Statement q(db,"UPDATE Node SET submap_pose=? WHERE id=1;");sqlite3_bind_blob(q.get(),1,m.data(),64,SQLITE_TRANSIENT);database_detail::step(db,q.get());}
      sqlite3_close(db);const auto unchanged=bytes(path);
      errorIs(MapErrorCode::HistoricalData,[&]{PoseGraphBackend b(config,{},path.string(),{});});
      check(!warning() && bytes(path)==unchanged,"invalid persisted pose hard failure preserves bytes");
      std::cout<<"PASS A2 actual persisted "<<(std::isnan(bad)?"NaN":"Inf")<<" pose hard failure\n";
    }
    spdlog::set_default_logger(old_logger);
  } catch(...) {reset();spdlog::set_default_logger(old_logger);throw;}
}
