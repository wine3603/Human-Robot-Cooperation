#include <chrono>
#include <thread>
#include <sstream>
#include <iostream>
#include "soem/ethercat.h"

#include "baichuan_fieldbus_ethercat/ethercat_master.h"

namespace baichuan
{

namespace fieldbus
{

EtherCATMaster::EtherCATMaster() : logger_(rclcpp::get_logger("BaichuanFieldbusEtherCAT")), mutex_()
{
    is_working_ = false;
    actual_map_size_ = 0;
    expected_wkc_ = 0;
    custom_config_hook_ = nullptr;
    for (int idx = 0; idx < 4096; ++idx)
        io_map_[idx] = 0;
}

EtherCATMaster::~EtherCATMaster()
{
    if (isWorking())
    {
        stopWorking();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Request INIT state for all slaves.
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    // 检查从站是否进入Init状态。
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_INIT)
        RCLCPP_WARN(logger_, "Cannot set EC_STATE_INIT.");

    ec_close();
    ec_slavecount = 0;
    RCLCPP_INFO(logger_, "Close EtherCAT master.");
}

void EtherCATMaster::setCustomConfigurationHook(CustomConfigurationHook hook)
{
    custom_config_hook_ = hook;
}

bool EtherCATMaster::registerErrorCallback(const std::string & name, EtherCATErrorCallback callback)
{
    if (registered_callbacks_.find(name) != registered_callbacks_.end())
    {
        RCLCPP_WARN(logger_, "Callback function with name '%s' has already been registered.", name.c_str());
        return false;
    }

    registered_callbacks_[name] = callback;
    RCLCPP_INFO(logger_, "Registered error callback function '%s'.", name.c_str());
    return true;
}

bool EtherCATMaster::unregisterErrorCallback(const std::string & name)
{
    if (registered_callbacks_.find(name) == registered_callbacks_.end())
    {
        RCLCPP_WARN(logger_,
                "Cannot unregister error callback for '%s' since it was not registered.", name.c_str());

        return false;
    }

    registered_callbacks_.erase(name);
    return true;
}

bool EtherCATMaster::init(const std::string & if_name)
{
    if_name_ = if_name;
    if (ec_init(if_name.c_str()) <= 0)
    {
        std::string message = "Cannot create EtherCAT connection on interface '" + if_name_ + 
                "', maybe you should call the function as root user.";
        RCLCPP_ERROR(logger_, message.c_str());
        return false;
    }

    if (ec_config_init(FALSE) > 0)
    {
        RCLCPP_INFO(logger_, "%d slaves were found on interface '%s'.", ec_slavecount, if_name_.c_str());
    }
    else
    {
        RCLCPP_ERROR(logger_, "No slave was found on interface '%s'.", if_name.c_str());
        return false;
    }

    // 检查从站是否进入PreOP状态。
    if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) != EC_STATE_PRE_OP)
    {
        RCLCPP_ERROR(logger_, "Cannot set EC_STATE_PRE_OP.");
        return false;
    }

    // 设置自定义配置函数，用于分配和映射RxPDO/TxPDO。
    if (custom_config_hook_)
    {
        RCLCPP_INFO(logger_, "Set up custom configuration hook.");
        for (int idx = 1; idx <= ec_slavecount; ++idx)
            ec_slave[idx].PO2SOconfig = custom_config_hook_;
    }

    // 映射到本地数据。
    actual_map_size_ = ec_config_map(&io_map_);
    RCLCPP_INFO(logger_, "Actual EtherCAT IOMap size: %d.", actual_map_size_);

    // 配置分布式时钟。
    if (!ec_configdc())
    {
        RCLCPP_ERROR(logger_, "Configuration of DC failed.");
        return false;
    }

    printSlaveInfo();

    // 检查从站是否进入SafeOP状态。SafeOP状态，PDO数据可读，写无效。
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP)
    {
        RCLCPP_ERROR(logger_, "Cannot set EC_STATE_SAFE_OP.");
        return false;
    }

    expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    RCLCPP_INFO(logger_, "Expected WKC: %d.", expected_wkc_);

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    // Send one valid process data to make outputs in slaves happy.
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // Request OP state for all slaves.
    ec_writestate(0);
    // Wait for all slaves to reach OP stat.
    int check_times = 100;
    do
    {
        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE*4);
    } while (check_times-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        RCLCPP_ERROR(logger_, "Cannot set EC_STATE_OPERATIONAL.");
        return false;
    }
    else
    {
        RCLCPP_INFO(logger_, "All slaves reached state EC_STATE_OPERATIONAL.");
        return true;
    }
}

void EtherCATMaster::printSlaveInfo()
{
    ec_readstate();
    std::stringstream stream;
    stream << "\n===== Slave Information =====" << std::endl;
    for(int idx_slave = 1 ; idx_slave <= ec_slavecount ; idx_slave++)
    {
        stream << "Slave: " << idx_slave << std::endl;
        stream << "  Name: " << ec_slave[idx_slave].name << std::endl;
        stream << "  Output size: " << (int)ec_slave[idx_slave].Obits << " bits" << std::endl;
        stream << "  Input size: " << (int)ec_slave[idx_slave].Ibits << " bits" << std::endl;
        stream << "  State: " << (int)ec_slave[idx_slave].state << std::endl;
        stream << "  Delay: " << (int)ec_slave[idx_slave].pdelay << std::endl;
        stream << "  Has DC: " << (int)ec_slave[idx_slave].hasdc << std::endl;

        if (ec_slave[idx_slave].hasdc)
            stream << "  DCParentport: " << (int)ec_slave[idx_slave].parentport << std::endl;

        stream << "  Active ports: " <<
                ((ec_slave[idx_slave].activeports & 0x01) > 0) << "."  <<
                ((ec_slave[idx_slave].activeports & 0x02) > 0) << "." <<
                ((ec_slave[idx_slave].activeports & 0x04) > 0) << "." <<
                ((ec_slave[idx_slave].activeports & 0x08) > 0) << std::endl;

        stream << "  Configured address: " << (int)ec_slave[idx_slave].configadr << std::endl;
        stream << "  Man: " << (int)ec_slave[idx_slave].eep_man << " ID: "  <<
                (int)ec_slave[idx_slave].eep_id << " Rev: " <<
                (int)ec_slave[idx_slave].eep_rev << std::endl;

        for(int idx_sm = 0 ; idx_sm < EC_MAXSM ; idx_sm++)
        {
            if(ec_slave[idx_slave].SM[idx_sm].StartAddr > 0)
            {
                stream << "  SM" << idx_sm <<
                        " A: " << etohs(ec_slave[idx_slave].SM[idx_sm].StartAddr) <<
                        " L: " << etohs(ec_slave[idx_slave].SM[idx_sm].SMlength) <<
                        " F: " << etohl(ec_slave[idx_slave].SM[idx_sm].SMflags) <<
                        " Type: " << (int)ec_slave[idx_slave].SMtype[idx_sm] << std::endl;
            }
        }

        for(int idx_fmmu = 0; idx_fmmu < ec_slave[idx_slave].FMMUunused; idx_fmmu++)
        {
            stream << "  FMMU" << idx_fmmu <<
                    " Ls: " << etohl(ec_slave[idx_slave].FMMU[idx_fmmu].LogStart) <<
                    " Ll: " << etohs(ec_slave[idx_slave].FMMU[idx_fmmu].LogLength) <<
                    " Lsb: " << (int)ec_slave[idx_slave].FMMU[idx_fmmu].LogStartbit <<
                    " Leb: " << (int)ec_slave[idx_slave].FMMU[idx_fmmu].LogEndbit <<
                    " Ps: " << etohs(ec_slave[idx_slave].FMMU[idx_fmmu].PhysStart) <<
                    " Psb: " << (int)ec_slave[idx_slave].FMMU[idx_fmmu].PhysStartBit <<
                    " Ty: " << (int)ec_slave[idx_slave].FMMU[idx_fmmu].FMMUtype <<
                    " Act: " << (int)ec_slave[idx_slave].FMMU[idx_fmmu].FMMUactive << std::endl;
        }

        stream << "  FMMUfunc 0: " << ec_slave[idx_slave].FMMU0func <<
                " 1: " << (int)ec_slave[idx_slave].FMMU1func <<
                " 2: " << (int)ec_slave[idx_slave].FMMU2func <<
                " 3: " << (int)ec_slave[idx_slave].FMMU3func << std::endl;
        stream << "  MBX length wr: " << (int)ec_slave[idx_slave].mbx_l <<
                " rd: " << (int)ec_slave[idx_slave].mbx_rl <<
                " MBX protocols: " << (int)ec_slave[idx_slave].mbx_proto << std::endl;

        int ssigen = ec_siifind(idx_slave, ECT_SII_GENERAL);
        /* SII general section */
        if (ssigen)
        {
            ec_slave[idx_slave].CoEdetails = ec_siigetbyte(idx_slave, ssigen + 0x07);
            ec_slave[idx_slave].FoEdetails = ec_siigetbyte(idx_slave, ssigen + 0x08);
            ec_slave[idx_slave].EoEdetails = ec_siigetbyte(idx_slave, ssigen + 0x09);
            ec_slave[idx_slave].SoEdetails = ec_siigetbyte(idx_slave, ssigen + 0x0a);
            if((ec_siigetbyte(idx_slave, ssigen + 0x0d) & 0x02) > 0)
            {
                ec_slave[idx_slave].blockLRW = 1;
                ec_slave[0].blockLRW++;
            }
            ec_slave[idx_slave].Ebuscurrent = ec_siigetbyte(idx_slave, ssigen + 0x0e);
            ec_slave[idx_slave].Ebuscurrent += ec_siigetbyte(idx_slave, ssigen + 0x0f) << 8;
            ec_slave[0].Ebuscurrent += ec_slave[idx_slave].Ebuscurrent;
        }

        stream << "  CoE details: " << (int)ec_slave[idx_slave].CoEdetails <<
                " FoE details: " << (int)ec_slave[idx_slave].FoEdetails <<
                " EoE details: "  << (int)ec_slave[idx_slave].EoEdetails <<
                " SoE details: " << (int)ec_slave[idx_slave].SoEdetails << std::endl;

        stream << "  Ebus current: " << ec_slave[idx_slave].Ebuscurrent << " mA" << std::endl;
        stream << "  Only LRD/LWR: " << (int)ec_slave[idx_slave].blockLRW << std::endl;
    }
    stream << "===== End of Slave Information =====";

    RCLCPP_INFO_STREAM(logger_, stream.str());
}

int EtherCATMaster::getActualMapSize()
{
    return actual_map_size_;
}

bool EtherCATMaster::startWorking(int rate)
{
    if (isWorking())
    {
        RCLCPP_WARN(logger_, "EtherCATMaster is already working, you should not call 'startWorking' again.");
        return true;
    }

    is_working_ = true;
    std::thread{[this, rate]
    {
        // 设置线程优先级。
        struct sched_param param;
        param.sched_priority = 90;
        pthread_setschedparam(pthread_self(), SCHED_RR, &param);

        int sleep_ms = 1000 / rate;
        int wkc_error_count = 0; // 连续10次WKC错误会终止通信。
        RCLCPP_INFO(logger_, "EtherCAT master will update every %d milliseconds.", sleep_ms);
        std::chrono::system_clock::time_point sleep_till = std::chrono::system_clock::now();
        while (rclcpp::ok())
        {
            sleep_till += std::chrono::milliseconds(sleep_ms);

            int wkc = 0;
            // Lock mutex.
            {
                std::scoped_lock<std::mutex> lock(mutex_);
                if (!is_working_)
                {
                    RCLCPP_WARN(logger_, "EtherCAT master will stop working since the working flag is set to false.");
                    return;
                }

                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);
            }

            if (expected_wkc_ != wkc)
            {
                // RCLCPP_WARN(logger_, "WKC {} is not equal to the expected {}.", wkc, expected_wkc_);
                wkc_error_count++;
            }
            else
            {
                wkc_error_count = 0;
            }

            if(wkc_error_count >= 10)
            {
                std::string msg = "Quitting communication because 10 continuous WKC errors happened, expected WKC=" + 
                        std::to_string(expected_wkc_) + ", but got " + std::to_string(wkc) + ".";
                RCLCPP_ERROR(logger_, msg.c_str());
                for (auto & node : registered_callbacks_)
                    node.second(msg);

                stopWorking();
                RCLCPP_ERROR(logger_, "Quitting EtherCAT PDO updating loop due to continuous WKC errors.");
                return;
            }
            std::this_thread::sleep_until(sleep_till);
        }

        RCLCPP_ERROR(logger_, "Quiting EtherCAT realtime update loop due to rclcpp not ok.");
    }}.detach();

    return true;
}

void EtherCATMaster::stopWorking()
{
    std::scoped_lock<std::mutex> lock(mutex_);
    is_working_ = false;
}

bool EtherCATMaster::isWorking()
{
    std::scoped_lock<std::mutex> lock(mutex_);
    return is_working_;
}

std::vector<uint8_t> EtherCATMaster::readIOMap(int idx_start, int length)
{
    std::scoped_lock<std::mutex> lock(mutex_);
    if ((idx_start + length) > actual_map_size_)
    {
        RCLCPP_WARN(logger_, 
                "(idx_start+length) exceeds the limit of IO map, part of the returned data might be meaningless.");
        RCLCPP_WARN(logger_,"idx_start:%d,length:%d,actual_map_size_:%d",idx_start,length,actual_map_size_);
    }

    std::vector<uint8_t> ret(length);
    for (std::size_t idx = 0; idx < std::min(ret.size(), std::size_t(actual_map_size_-idx_start)); ++idx)
        ret[idx] = io_map_[idx_start+idx];

    return ret;
}

void EtherCATMaster::writeIOMap(int idx_start, const std::vector<uint8_t> & data)
{
    std::scoped_lock<std::mutex> lock(mutex_);
    if ((idx_start+data.size()) > actual_map_size_)
    {
        RCLCPP_WARN(logger_, 
                "(idx_start+data.size()) exceeds the limit of IO map, part of 'data' might not be written into IO map.");
    }

    for (std::size_t idx = 0; idx < std::min(data.size(), std::size_t(actual_map_size_-idx_start)); ++idx)
        io_map_[idx_start+idx] = data[idx];
}

std::vector<uint8_t> EtherCATMaster::readSDO(int slave, int index, int subindex, int byte_size)
{
    int size = byte_size;   // 实际读到的字节数会返回。
    std::vector<uint8_t> data(size);
    ec_SDOread(slave, index, subindex, FALSE, &size, data.data(), EC_TIMEOUTSAFE);
    std::vector<uint8_t> ret(size);
    for (int idx = 0; idx < size; ++idx)
        ret[idx] = data[idx];

    return ret;
}

void EtherCATMaster::writeSDO(int slave, int index, int subindex, std::vector<uint8_t> data)
{
    int byte_size = data.size();
    ec_SDOwrite(slave, index, subindex, FALSE, byte_size, data.data(), EC_TIMEOUTSAFE);
}
void EtherCATMaster::writeManager(int slave_no, uint8_t channel, uint8_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("writeManager"),"Before Lock");
    std::scoped_lock<std::mutex> lock(mutex_);
    RCLCPP_INFO(rclcpp::get_logger("writeManager"),"After Lock");

  if (slave_no > ec_slavecount) {
      RCLCPP_ERROR(logger_,"ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
      exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Obits) {
      RCLCPP_ERROR(logger_,"ERROR : slave_no(%d) : channel(%d) is larger than Output bits (%d), you may need to read elfin_robot/docs/Fix_ESI.md or elfin_robot/docs/Fix_ESI_english.md with a Markdown editor or on github.com\n", slave_no, channel*8, ec_slave[slave_no].Obits);
      exit(1);
  }
  ec_slave[slave_no].outputs[channel] = value;
}
uint8_t EtherCATMaster::readOutputManager(int slave_no, uint8_t channel) 
{
  std::scoped_lock<std::mutex> lock(mutex_);
  if (slave_no > ec_slavecount) {
    RCLCPP_ERROR(logger_,"ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Obits) {
    RCLCPP_ERROR(logger_,"ERROR : slave_no(%d) : channel(%d) is larger than Output bits (%d), you may need to read elfin_robot/docs/Fix_ESI.md or elfin_robot/docs/Fix_ESI_english.md with a Markdown editor or on github.com\n", slave_no, channel*8, ec_slave[slave_no].Obits);
    exit(1);
  }
  return ec_slave[slave_no].outputs[channel];
}

uint8_t EtherCATMaster::readInputManager(int slave_no, uint8_t channel) 
{
  std::scoped_lock<std::mutex> lock(mutex_);
  if (slave_no > ec_slavecount) {
    RCLCPP_ERROR(logger_,"ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Ibits) {
    RCLCPP_ERROR(logger_,"ERROR : slave_no(%d) : channel(%d) is larger than Input bits (%d), you may need to read elfin_robot/docs/Fix_ESI.md or elfin_robot/docs/Fix_ESI_english.md with a Markdown editor or on github.com\n", slave_no, channel*8, ec_slave[slave_no].Ibits);
    exit(1);
  }
  return ec_slave[slave_no].inputs[channel];
}

} // namespace fieldbus

} // namespace baichuan
