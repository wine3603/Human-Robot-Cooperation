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
#include "unitree_sdk2_bridge/unitree_sdk2_bridge.h"
#include <pthread.h>
#include "yaml-cpp/yaml.h"

#include "param.h"

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

namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr; //描述机器人结构、关节、惯量等静态参数
  mjData *d = nullptr; //描述当前的状态（qpos, qvel, ctrl 等）

  // control noise variables
  mjtNum *ctrlnoise = nullptr; //控制噪声

  struct SimulationConfig            //仿真配置结构体
  {
    std::string robot = "go2";
    std::string robot_scene = "scene.xml";

    int domain_id = 1;
    std::string interface = "lo";

    int use_joystick = 0;
    std::string joystick_type = "xbox";
    std::string joystick_device = "/dev/input/js0";
    int joystick_bits = 16;

    int print_scene_information = 1;

    int enable_elastic_band = 0;
    int band_attached_link = 0;

  } config;

  using Seconds = std::chrono::duration<double>; //声明一个 double 类型的时间段

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

  mjModel *LoadModel(const char *file, mj::Simulate &sim)   //加载mjcf模型
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength]; //把传入的 file 参数拷贝到本地 filename 数组中
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = ""; 
    mjModel *mnew = 0;                 //加载模型：支持 .mjb 和 .xml 格式
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
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    } //判断文件格式是.mjb和.xml，并按照格式加载。

    mju::strcpy_arr(sim.load_error, loadError); //保存错误信息到 simulate 状态中

    if (!mnew) //加载失败
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0]) //成功加载模型，但发出警告；打印出来，并设置 sim.run = 0，暂停仿真器，等待用户确认
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim) //不断推进 MuJoCo 仿真、控制仿真频率、处理模型热加载、注入控制噪声、弹性带力、以及保存仿真历史。
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // ChannelFactory::Instance()->Init(0);
    // UnitreeDds ud(d);

    // run until asked to exit
    while (!sim.exitrequest.load())//只要没有收到退出请求，就一直执行仿真循环
    {
      if (sim.droploadrequest.load())  //文件拖拽加载 .xml/.mjb 模型； 检查是否用户拖了一个文件进来
      {
        sim.LoadMessage(sim.dropfilename);  // 在窗口显示“正在加载文件”的消息
        mjModel *mnew = LoadModel(sim.dropfilename, sim); //把文件解析成新的模型指针 mnew
        sim.droploadrequest.store(false); //标记这个拖拽请求已经处理完了

        mjData *dnew = nullptr; //清除data
        if (mnew)
          dnew = mj_makeData(mnew); //创建新的模型data
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename); // 把新模型和数据加载进 Simulate 类中

          mj_deleteData(d);                       //删除旧的模型
          mj_deleteModel(m);                      //删除旧的模型数据

          m = mnew;                               //更新模型
          d = dnew;                               //更新数据
          mj_forward(m, d);                       //计算一次前向动力学，但不推进仿真时间 

          // allocate ctrlnoise
          free(ctrlnoise);                        // 清空原来的噪声缓存
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);//为每个控制器分配新的噪音缓存。malloc(...)：分配一段连续内存，大小是 控制维度数 * 每个控制变量的大小。
          mju_zero(ctrlnoise, m->nu);       //清零并初始化
        }
        else
        {
          sim.LoadMessageClear();         // 清除“加载中”的提示
        }
      }

      if (sim.uiloadrequest.load()) //使用UI加载模型
      {
        sim.uiloadrequest.fetch_sub(1); //把请求次数减一，避免重复处理
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
          mj_forward(m, d);

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
      if (sim.run && sim.busywait)  //sim.busywait 是否采用 忙等待
      { 
        std::this_thread::yield(); //响应更及时，高CPU占用
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));//当前线程休眠 1 毫秒；
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (sim.run) //没有暂停为
          {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now(); //上次同步时 CPU 的时间

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;   //距离上次同步，CPU 过去了多久
            double elapsedSim = d->time - syncSim; //仿真时间前进了多少

            // inject noise
            if (sim.ctrl_noise_std)          //如果添加控制噪声
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

                // 打印当前控制输入（噪声值）
                printf("  ctrl[%d] = %.4f\n", i, d->ctrl[i]);
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned = //如果“现实时间”和“仿真时间”错位了（超过 syncMisalign），就说明“失步”，需要重新同步
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
              mj_step(m, d);
              stepped = true;
            } //如果失步，就 mj_step 一次并重设同步

            // in-sync: step until ahead of cpu
            else //否则进入“流畅推进”模式
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) //仿真时间 < 现实时间 / slowdown && 当前消耗时间 < refreshTime限制
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // elastic band on base link
                if (sim.use_elastic_band_ == 1)//给机器人施加弾力挂带
                {
                  if (sim.elastic_band_.enable_)
                  {
                    vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]}; //读取当前机器人位置
                    vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};//读取当前机器人速度

                    sim.elastic_band_.Advance(x, dx);      //调用 Advance 算法算出挂带弹力

                    d->xfrc_applied[config.band_attached_link] = sim.elastic_band_.f_[0];           //X方向力
                    d->xfrc_applied[config.band_attached_link + 1] = sim.elastic_band_.f_[1];       //Y方向力
                    d->xfrc_applied[config.band_attached_link + 2] = sim.elastic_band_.f_[2];       //Z方向力
                  }
                }

                // call mj_step
                mj_step(m, d); //前向动力学（计算加速度 qacc） 数值积分（更新状态 qpos, qvel） 处理碰撞和约束 更新传感器数据
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped) //步进完成后记录历史轨迹
            {
              sim.AddToHistory();
            }
          }

          // paused
          else       //暂停时逻辑
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
        }
      } // release std::lock_guard<std::mutex>
    }
  }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)           //如果给定了模型名称
  {
    sim->LoadMessage(filename);  //GUI 显示“loading...”
    m = LoadModel(filename, *sim); //加载模型
    if (m)
      d = mj_makeData(m);  //初始化数据结构
    if (d)
    {
      sim->Load(m, d, filename);
      mj_forward(m, d);

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
  mj_deleteModel(m);//清理内存资源

  exit(0);
}

void *UnitreeSdk2BridgeThread(void *arg)
{
  // Wait for mujoco data
  while (1)
  {
    if (d)
    {
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(500000);//阻塞等待机制，确保 MuJoCo 仿真中的 mjData* d 已经初始化
  }

  if (config.robot == "h1" || config.robot == "g1")
  {
    config.band_attached_link = 6 * mj_name2id(m, mjOBJ_BODY, "torso_link");
  }
  else
  {
    config.band_attached_link = 6 * mj_name2id(m, mjOBJ_BODY, "base_link");
  }

  ChannelFactory::Instance()->Init(config.domain_id, config.interface);  //初始化 DDS 通信（ROS2 数据流）
  UnitreeSdk2Bridge unitree_interface(m, d); //使用m和d构造一个UnitreeSdk2Bridge类，用来桥接unitreeSDK

  if (config.use_joystick == 1) //初始化手柄（如果配置里开启）
  {
    unitree_interface.SetupJoystick(config.joystick_device, config.joystick_type, config.joystick_bits);
  }

  if (config.print_scene_information == 1) //打印场景信息
  {
    unitree_interface.PrintSceneInformation();
  }

  unitree_interface.Run(); //启动主循环：开始收发数据、同步状态

  pthread_exit(NULL); //退出线程
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
  std::printf("MuJoCo version %s\n", mj_versionString());//打印版本信息
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");//版本不一致直接退出
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();//自动查找 plugin/ 目录中自定义 .so / .dll 插件，动态加载进来。

  mjvCamera cam;
  mjv_defaultCamera(&cam);// 初始化摄像机

  mjvOption opt;
  mjv_defaultOption(&opt);// 初始化渲染参数

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);// 初始化扰动参数

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false); //使用了 MuJoCo 提供的 mj::Simulate 封装类创建仿真器对象

  // Load simulation configuration
  YAML::Node yaml_node = YAML::LoadFile("../config.yaml");   //加载yaml文件
  config.robot = yaml_node["robot"].as<std::string>();
  config.robot_scene = yaml_node["robot_scene"].as<std::string>();
  config.domain_id = yaml_node["domain_id"].as<int>();
  config.interface = yaml_node["interface"].as<std::string>();
  config.print_scene_information = yaml_node["print_scene_information"].as<int>();
  config.enable_elastic_band = yaml_node["enable_elastic_band"].as<int>();
  config.use_joystick = yaml_node["use_joystick"].as<int>();
  config.joystick_type = yaml_node["joystick_type"].as<std::string>();
  config.joystick_device = yaml_node["joystick_device"].as<std::string>();
  config.joystick_bits = yaml_node["joystick_bits"].as<int>(); //读取yaml信息

  sim->use_elastic_band_ = config.enable_elastic_band;
  yaml_node.~Node();

  string scene_path = "../../unitree_robots/" + config.robot + "/" + config.robot_scene;//组合模型路径
  const char *filename = nullptr;
  if (argc > 1)
  {
    filename = argv[1]; //通过命令行传入 .xml 模型文件路径；如果没有，就用 YAML 中的默认路径。
  }
  else
  {
    filename = scene_path.c_str();
  }

  pthread_t unitree_thread; //线程标识符类型，专门用来保存某个线程的“句柄”或“ID”
  int rc = pthread_create(&unitree_thread, NULL, UnitreeSdk2BridgeThread, NULL);//创建 Unitree SDK 线程，线程调用的函数为UnitreeSdk2BridgeThread
  if (rc != 0) //通信接口创建失败
  {
    std::cout << "Error:unable to create thread," << rc << std::endl;
    exit(-1);
  }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);//启动仿真计算线程，线程调用了PhysicsThread，执行mujoco的仿真计算
  // start simulation UI loop (blocking call)
  sim->RenderLoop();//启动 GUI 主循环（阻塞）
  physicsthreadhandle.join();//等待线程结束

  pthread_exit(NULL);
  return 0;
}
