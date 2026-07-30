// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// !!! hack code: make glfw_adapter.window_ public
#define private public
#include "glfw_adapter.h"
#undef private

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "simulate.h"
#include "array_safety.h"
#include "unitree_sdk2_bridge.h"
#include "param.h"
#include "dpcbf/dynamic_obstacles.h"
#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf/dpcbf_visualizer.h"

// PERCEPTION HOOK 0/5 - optional subsystem. PERCEPTION_ENABLED comes from the
// perception_integration target, which only exists when PERCEPTION_ENABLE=ON. With it OFF,
// every hook region below compiles away to nothing.
#ifdef PERCEPTION_ENABLED
#include "perception/integration/mid360_bringup.h"
#include "perception/integration/obstacle_source_selector.h"
#endif

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"
#define NUM_MOTOR_IDL_GO 20

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

class ElasticBand
{
public:
  ElasticBand(){};
  void Advance(std::vector<double> x, std::vector<double> dx)
  {
    std::vector<double> delta_x = {0.0, 0.0, 0.0};
    delta_x[0] = point_[0] - x[0];
    delta_x[1] = point_[1] - x[1];
    delta_x[2] = point_[2] - x[2];
    double distance = sqrt(delta_x[0] * delta_x[0] + delta_x[1] * delta_x[1] + delta_x[2] * delta_x[2]);

    std::vector<double> direction = {0.0, 0.0, 0.0};
    direction[0] = delta_x[0] / distance;
    direction[1] = delta_x[1] / distance;
    direction[2] = delta_x[2] / distance;

    double v = dx[0] * direction[0] + dx[1] * direction[1] + dx[2] * direction[2];

    f_[0] = (stiffness_ * (distance - length_) - damping_ * v) * direction[0];
    f_[1] = (stiffness_ * (distance - length_) - damping_ * v) * direction[1];
    f_[2] = (stiffness_ * (distance - length_) - damping_ * v) * direction[2];
  }


  double stiffness_ = 200;
  double damping_ = 100;
  std::vector<double> point_ = {0, 0, 3};
  double length_ = 0.0;
  bool enable_ = true;
  std::vector<double> f_ = {0, 0, 0};
};
inline ElasticBand elastic_band;


namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;

  dpcbf::DynamicObstacleManager dynamic_obstacles;
  dpcbf::DpcbfSafetyFilter safety_filter;
  dpcbf::DpcbfVisualizer dpcbf_visualizer;

  // PERCEPTION HOOK 1/5 - the single facade the application constructs.
#ifdef PERCEPTION_ENABLED
  perception::integration::Mid360BringUp perception_mid360;

  // Decides what DPCBF is fed. In the shipped `oracle` mode it wraps exactly the
  // DynamicObstacleManager::Snapshot() conversion that used to live inline in the axis-filter
  // lambda below (HOOK 5/5) - same obstacles, same order, same numbers, proven bit-for-bit by
  // perception_oracle_equivalence_test against fixtures captured from the pre-change code.
  //
  // Deliberately independent of perception_mid360: the oracle source needs no LiDAR, no
  // mjModel and no scan, so `perception.enabled: false` or an unbindable sensor must not
  // disturb the obstacle path. It is also runner-less and thread-less - it runs synchronously
  // on whatever thread calls it, which for now is only the bridge thread.
  perception::integration::ObstacleSourceSelector perception_obstacle_source;

  // Rebinds perception to a freshly loaded (model, data) pair and cross-checks the
  // compiled MJCF <site> against perception.lidar. Called at each of the three model-load
  // sites, mirroring dynamic_obstacles.BindModel.
  //
  // The extrinsic disagreement is a HARD FAILURE by design: the closed loop drives the
  // sensor from the MJCF site while config, tests and any headless harness compose it from
  // the YAML, and nothing structural keeps the two in step. Continuing past a mismatch
  // means shipping quietly wrong perception, so the process exits non-zero instead.
  void RebindPerception(mj::Simulate &sim)
  {
    if (m == nullptr || d == nullptr)
    {
      return;
    }

    // The obstacle source first, and outside the `enabled()` gate above: it is bound to the
    // DynamicObstacleManager at startup (HOOK 4/5) and needs the model only to resolve the
    // robot base body for its ground-truth pose leg. That leg is diagnostic - nothing on the
    // live filter path reads it - so a failure here is reported and survived rather than
    // fatal, unlike the extrinsic guard below. The obstacle states DPCBF actually consumes do
    // not depend on it and keep flowing.
    if (perception_obstacle_source.bound())
    {
      std::string obstacle_error;
      if (!perception_obstacle_source.BindModel(m, obstacle_error))
      {
        std::cerr << "[perception] oracle robot-pose binding unavailable: " << obstacle_error
                  << std::endl;
      }
    }

    if (!perception_mid360.enabled())
    {
      return;
    }

    std::string message;
    if (!perception_mid360.VerifyExtrinsic(m, message))
    {
      std::cerr << message << std::endl;
      std::exit(EXIT_FAILURE);
    }
    std::cout << "[perception] " << message << std::endl;

    // The viewer reads user_scn from the render thread, so swap it under sim.mtx.
    const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
    sim.user_scn = nullptr;
    std::string error;
    if (!perception_mid360.BindModel(m, d, error))
    {
      std::cerr << "FATAL: perception bind failed: " << error << std::endl;
      std::exit(EXIT_FAILURE);
    }
    sim.user_scn = perception_mid360.overlay_scene();
    perception_mid360.StartLiveView();
  }
#endif

  // control noise variables
  mjtNum *ctrlnoise = nullptr;

  using Seconds = std::chrono::duration<double>;

  double AxisToCommand(double axis, double minimum, double maximum)
  {
    const double normalized = std::clamp(axis, -1.0, 1.0);
    return normalized >= 0.0 ? normalized * maximum : -normalized * minimum;
  }

  float CommandToAxis(double command, double minimum, double maximum)
  {
    if (command >= 0.0)
    {
      return static_cast<float>(maximum > 0.0 ? command / maximum : 0.0);
    }
    return static_cast<float>(minimum < 0.0 ? -command / minimum : 0.0);
  }

  dpcbf::RobotState ReadRobotGroundTruth(const mjModel* model, const mjData* data,
                                         int body_id)
  {
    dpcbf::RobotState state;
    if (!model || !data || body_id < 0)
    {
      return state;
    }
    const mjtNum* position = data->xpos + 3 * body_id;
    const mjtNum* rotation = data->xmat + 9 * body_id;
    state.x = position[0];
    state.y = position[1];
    state.phi = std::atan2(rotation[3], rotation[0]);
    mjtNum body_velocity[6] = {};
    mj_objectVelocity(model, data, mjOBJ_BODY, body_id, body_velocity, 1);
    state.sagittal_velocity = body_velocity[3];
    state.lateral_velocity = body_velocity[4];
    return state;
  }

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {
#if defined(_WIN32) || defined(__CYGWIN__)
    constexpr char kPathSep = '\\';
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      DWORD buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
        if (written < buf_size)
        {
          success = true;
        }
        else if (written == buf_size)
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
        else
        {
          std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
          return "";
        }
      }
      return realpath.get();
    }();
#else
    constexpr char kPathSep = '/';
#if defined(__APPLE__)
    std::unique_ptr<char[]> buf(nullptr);
    {
      std::uint32_t buf_size = 0;
      _NSGetExecutablePath(nullptr, &buf_size);
      buf.reset(new char[buf_size]);
      if (!buf)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }
      if (_NSGetExecutablePath(buf.get(), &buf_size))
      {
        std::cerr << "unexpected error from _NSGetExecutablePath\n";
      }
    }
    const char *path = buf.get();
#else
    const char *path = "/proc/self/exe";
#endif
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();
#endif

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }

    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }

    // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
#else
    const std::string sep = "/";
#endif

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mjSpec* spec = mj_parseXML(filename, nullptr, loadError, kErrorLength);
      if (spec)
      {
        try
        {
          dynamic_obstacles.AddToSpec(spec);
          mnew = mj_compile(spec, nullptr);
          if (!mnew)
          {
            mju::strcpy_arr(loadError, mjs_getError(spec));
          }
        }
        catch (const std::exception& error)
        {
          mju::strcpy_arr(loadError, error.what());
        }
        mj_deleteSpec(spec);
      }
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // ChannelFactory::Instance()->Init(0);
    // UnitreeDds ud(d);

    // run until asked to exit
    while (!sim.exitrequest.load())
    {
      if (sim.droploadrequest.load())
      {
        sim.LoadMessage(sim.dropfilename);
        mjModel *mnew = LoadModel(sim.dropfilename, sim);
        sim.droploadrequest.store(false);

        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          dynamic_obstacles.BindModel(m, d);
          mj_forward(m, d);

          // PERCEPTION HOOK 2/5 - model-load site 1 of 3 (drag-and-drop load).
#ifdef PERCEPTION_ENABLED
          RebindPerception(sim);
#endif

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      if (sim.uiloadrequest.load())
      {
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          dynamic_obstacles.BindModel(m, d);
          mj_forward(m, d);

          // PERCEPTION HOOK 2/5 - model-load site 2 of 3 (UI reload).
#ifdef PERCEPTION_ENABLED
          RebindPerception(sim);
#endif

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (sim.run)
          {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std)
            {
              // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
              mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
              mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

              for (int i = 0; i < m->nu; i++)
              {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed)
            {
              // re-sync
              syncCPU = startCPU;
              syncSim = d->time;
              sim.speed_changed = false;

              // run single step, let next iteration deal with timing
              dynamic_obstacles.Step(m, d, m->opt.timestep);
              mj_step(m, d);
              stepped = true;
            }

            // in-sync: step until ahead of cpu
            else
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // elastic band on base link
                if (param::config.enable_elastic_band == 1)
                {
                  if (elastic_band.enable_)
                  {
                    std::vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]};
                    std::vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};

                    elastic_band.Advance(x, dx);

                    d->xfrc_applied[param::config.band_attached_link] = elastic_band.f_[0];
                    d->xfrc_applied[param::config.band_attached_link + 1] = elastic_band.f_[1];
                    d->xfrc_applied[param::config.band_attached_link + 2] = elastic_band.f_[2];
                  }
                }

                // call mj_step
                dynamic_obstacles.Step(m, d, m->opt.timestep);
                mj_step(m, d);
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped)
            {
              sim.AddToHistory();
            }
          }

          // paused
          else
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }

          // PERCEPTION HOOK 3/5 - the ray-sampling call.
          //
          // Deliberately here and nowhere else: this is inside the physics loop's existing
          // sim.mtx critical section, which is the only place a consistent mjData exists,
          // and it is after stepping so the scan sees the post-step world. One call covers
          // both the re-sync and the in-sync stepping branches. When the viewer is paused
          // sim time is frozen, so `force` re-scans on a wall-clock cadence instead, which
          // keeps the live overlay responsive while dragging bodies around.
          //
          // KNOWN ONE-TIMESTEP PHASE OFFSET. After mj_step, d->xpos / d->site_xpos still
          // describe the pre-integration state while d->time has already advanced, and the
          // running branch above does not call mj_forward (only the paused branch does). The
          // scan geometry therefore lags its own timestamp by one timestep - measured at
          // 4.7 mm while falling at 2.4 m/s by perception_mid360_pose_tracking_test (V6c).
          // Left as-is deliberately: the right fix is to record the true acquisition time,
          // which is the deskew phase's job, not to force a kinematics refresh from the
          // perception path. It is ~50x smaller than the 100 ms scan window deskew must
          // handle regardless.
#ifdef PERCEPTION_ENABLED
          perception_mid360.MaybeScan(d->time, /*force=*/!sim.run);
#endif
        }
      } // release std::lock_guard<std::mutex>
    }
  }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      sim->Load(m, d, filename);
      dynamic_obstacles.BindModel(m, d);
      mj_forward(m, d);

      // PERCEPTION HOOK 2/5 - model-load site 3 of 3 (initial startup load).
#ifdef PERCEPTION_ENABLED
      RebindPerception(*sim);
#endif

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);

  exit(0);
}

void *UnitreeSdk2BridgeThread(void *arg)
{
  // Wait for mujoco data
  while (true)
  {
    if (d)
    {
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(500000);
  }

  unitree::robot::ChannelFactory::Instance()->Init(param::config.domain_id, param::config.interface);


  int body_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
  if (body_id < 0) {
    body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
  }
  param::config.band_attached_link = 6 * body_id;

  int dpcbf_body_id = mj_name2id(m, mjOBJ_BODY, safety_filter.base_body_name().c_str());
  if (dpcbf_body_id < 0) {
    std::cerr << "DPCBF base body was not found: "
              << safety_filter.base_body_name() << std::endl;
    return nullptr;
  }
  try {
    safety_filter.Initialize(m->opt.timestep);
    dpcbf_visualizer.Start();
  } catch (const std::exception& error) {
    std::cerr << "Failed to initialize DPCBF safety filter: " << error.what() << std::endl;
    return nullptr;
  }

  JoystickAxisFilter axis_filter = [dpcbf_body_id](float lx, float ly, float rx) {
    dpcbf::VelocityCommand desired;
    desired.sagittal = AxisToCommand(ly, safety_filter.sagittal_velocity_min(),
                                     safety_filter.sagittal_velocity_max());
    desired.lateral = AxisToCommand(-lx, safety_filter.lateral_velocity_min(),
                                    safety_filter.lateral_velocity_max());
    desired.yaw_rate = AxisToCommand(-rx, safety_filter.yaw_rate_min(),
                                     safety_filter.yaw_rate_max());

    // PERCEPTION HOOK 5/5 - the obstacle source.
    //
    // This is the sole DPCBF input site, which is why the hook has to be in the lambda body
    // itself: a lambda cannot be interposed from outside. In the shipped `oracle` mode the
    // call below is the identical conversion that used to be written out here - the same
    // Snapshot(), the same field copies, the same index-derived ids, the same order, the same
    // complete and uncapped list - relocated into ObstacleSourceSelector so the choice of
    // source is explicit and testable. perception_oracle_equivalence_test proves the
    // equivalence bit-for-bit, through the real DpcbfSafetyFilter, against fixtures captured
    // from this code before it moved.
    //
    // The #else branch is not a fallback, it is the original text: with PERCEPTION_ENABLE=OFF
    // the whole subsystem compiles away and the binary must stay byte-identical to a
    // repository that never had perception in it. It is also the rollback - deleting the
    // #ifdef arm restores the pre-phase lambda exactly.
    //
    // NOTE that ReadRobotGroundTruth below is deliberately NOT routed through the selector,
    // even though OracleProviderMj can produce the same pose. Moving it would add a second
    // mjData read on the bridge thread to a phase whose entire promise is "nothing changes",
    // for no benefit: the selector's robot-pose leg exists for the P11 evaluator, not for the
    // live filter. It also keeps this phase clear of the pre-existing, documented m/d race
    // between this thread and the physics thread - perception adds no reads here.
    std::vector<dpcbf::ObstacleState> obstacle_states;
#ifdef PERCEPTION_ENABLED
    obstacle_states = perception_obstacle_source.GetObstacleStates(d->time);
#else
    const auto obstacle_snapshot = dynamic_obstacles.Snapshot();
    obstacle_states.reserve(obstacle_snapshot.size());
    for (std::size_t obstacle_id = 0; obstacle_id < obstacle_snapshot.size();
         ++obstacle_id) {
      const auto& obstacle = obstacle_snapshot[obstacle_id];
      obstacle_states.push_back({obstacle.position[0], obstacle.position[1], obstacle.radius,
                                 obstacle.velocity[0], obstacle.velocity[1],
                                 static_cast<int>(obstacle_id)});
    }
#endif
    const dpcbf::RobotState robot = ReadRobotGroundTruth(m, d, dpcbf_body_id);
    const auto filtered = safety_filter.Filter(robot, desired, obstacle_states);
    dpcbf_visualizer.Update(robot, obstacle_states, filtered);
    return std::array<float, 3>{
        -CommandToAxis(filtered.command.lateral,
                       safety_filter.lateral_velocity_min(),
                       safety_filter.lateral_velocity_max()),
        CommandToAxis(filtered.command.sagittal,
                      safety_filter.sagittal_velocity_min(),
                      safety_filter.sagittal_velocity_max()),
        -CommandToAxis(filtered.command.yaw_rate,
                       safety_filter.yaw_rate_min(),
                       safety_filter.yaw_rate_max())};
  };
  
  std::unique_ptr<UnitreeSDK2BridgeBase> interface = nullptr;
  if (m->nu > NUM_MOTOR_IDL_GO) {
    interface = std::make_unique<G1Bridge>(m, d, axis_filter);
  } else {
    interface = std::make_unique<Go2Bridge>(m, d, axis_filter);
  }
  interface->start();
  
  while (true)
  {
    sleep(1);
  }
}
//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char *msg)
{
  rosetta_error_msg = msg;
}
#endif

// user keyboard callback
void user_key_cb(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act==GLFW_PRESS)
  {
    if(param::config.enable_elastic_band == 1) {
      if (key==GLFW_KEY_9) {
        elastic_band.enable_ = !elastic_band.enable_;
      } else if (key==GLFW_KEY_7 || key==GLFW_KEY_UP) {
        elastic_band.length_ -= 0.1;
      } else if (key==GLFW_KEY_8 || key==GLFW_KEY_DOWN) {
        elastic_band.length_ += 0.1;
      }
    }
    if(key==GLFW_KEY_BACKSPACE) {
      mj_resetData(m, d);
      dynamic_obstacles.Reset(m, d);
      mj_forward(m, d);
    }
  }
}

// run event loop
int main(int argc, char **argv)
{

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg)
  {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // Load simulation configuration
  std::filesystem::path proj_dir = std::filesystem::path(getExecutableDir()).parent_path();
  param::config.load_from_yaml(proj_dir / "config.yaml");
  try {
    const auto dpcbf_config = proj_dir.parent_path() / "dpcbf/config/dpcbf_config.yaml";
    dynamic_obstacles.LoadConfig(dpcbf_config);
    safety_filter.LoadConfig(dpcbf_config);
    dpcbf_visualizer.LoadConfig(dpcbf_config);
  } catch (const std::exception& error) {
    std::cerr << "Failed to load dynamic obstacle configuration: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  // PERCEPTION HOOK 4/5 - configuration. Its own file, so the perception schema never
  // leaks into dpcbf_config.yaml and dpcbf/ keeps taking zero edits.
#ifdef PERCEPTION_ENABLED
  try {
    perception_mid360.LoadConfig(proj_dir.parent_path() / "perception/configs/perception.yaml");
  } catch (const std::exception& error) {
    std::cerr << "Failed to load perception configuration: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  // Bind the obstacle source here, on the main thread, before either the bridge or the physics
  // thread starts. Two reasons it is here and not at model-load time:
  //   - In oracle mode it needs no mjModel. DynamicObstacleManager fills its obstacle list in
  //     AddToSpec, before compile, so Snapshot() is already meaningful. Binding now means the
  //     axis-filter lambda can never race a half-bound selector, whatever order the threads
  //     come up in.
  //   - An unimplemented mode must stop the process at startup, next to the config that
  //     selected it, rather than throwing out of a filter callback mid-run.
  {
    std::string error;
    if (!perception_obstacle_source.Bind(perception_mid360.config(), &dynamic_obstacles,
                                         error)) {
      std::cerr << "Failed to bind the perception obstacle source: " << error << '\n';
      return EXIT_FAILURE;
    }
    std::cout << "[perception] obstacle source: "
              << perception::integration::ToString(perception_obstacle_source.mode()) << '\n';
  }
#endif

  param::helper(argc, argv);
  if(param::config.robot_scene.is_relative()) {
    param::config.robot_scene = proj_dir.parent_path() / param::config.robot_scene;
  }

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ false);

  std::thread unitree_thread(UnitreeSdk2BridgeThread, nullptr);

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), param::config.robot_scene.c_str());
  // start simulation UI loop (blocking call)
  glfwSetKeyCallback(static_cast<mj::GlfwAdapter*>(sim->platform_ui.get())->window_,user_key_cb);
  sim->RenderLoop();
  physicsthreadhandle.join();

  pthread_exit(NULL);
  return 0;
}
