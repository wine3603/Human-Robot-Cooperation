#ifndef BAICHUAN_FIELDBUS_ETHERCAT_MASTER_H
#define BAICHUAN_FIELDBUS_ETHERCAT_MASTER_H

#include <unistd.h>

#include <map>
#include <mutex>
#include <vector>
#include <string>
#include <functional>

#include <rclcpp/logging.hpp>

namespace baichuan
{

namespace fieldbus
{

//!< 自定义配置函数的定义，由于底层SOEM是用C写的，此处需要使用C语言形式的函数指针。
typedef int (* CustomConfigurationHook) (uint16_t slave);   
//!< 总线错误时回调函数的定义。
using EtherCATErrorCallback = std::function<void(const std::string & msg)>;

class EtherCATMaster
{
public:
    EtherCATMaster();
    ~EtherCATMaster();
    EtherCATMaster(EtherCATMaster const &) = delete;
    EtherCATMaster & operator=(EtherCATMaster const &) = delete;

    /**
     * @brief 设置自定义配置函数，用于分配和映射PDO。从站从PreOP状态转换到SafeOP状态时，会调用该函数。。
     * @param hook 自定义配置函数。
     */
    void setCustomConfigurationHook(CustomConfigurationHook hook);

    /**
     * @brief 注册错误回调函数，EtherCAT通信发生错误时调用该函数。
     * @param name 名称。
     * @param callback 回调函数。
     * @return 是否成功。
     */
    bool registerErrorCallback(const std::string & name, EtherCATErrorCallback callback);
    bool unregisterErrorCallback(const std::string & name);

    /**
     * @brief 初始化SOEM，扫描从站；完成PDO映射，使从站设备进入Operate状态，准备开始周期性通信。
     * @param if_name 网络接口名称。
     * @return
     */
    bool init(const std::string & if_name);

    /**
     * @brief 获取实际映射的字节长度。
     * @return 实际映射的字节长度。
     */
    int getActualMapSize();

    /**
     * @brief 输出从站信息。
     */
    void printSlaveInfo();

    /**
     * @brief 开始周期性通信。
     * @param rate 通信频率，默认是1000Hz。
     * @return 成功与否。
     */
    bool startWorking(int rate = 1000);
    void stopWorking();
    bool isWorking();

    /**
     * @brief 获取IO映射数据，用于读取PDO数据。如果(idx_start+length)大于io_map_的长度，则只读取到io_map_的最后一个元素。
     * @param idx_start 起始索引，从0开始，包含该索引。
     * @param length 读取的数据长度，实际读到的数据长度以返回的数组的长度为准。
     * @return IO映射中的字节数组。
     */
    std::vector<uint8_t> readIOMap(int idx_start, int length);

    inline uint8_t * getIOMapPtr() { return io_map_; }

    /**
     * @brief 写入IO映射数据，用于写入PDO数据。如果(idx_start+data.size())大于io_map_的长度，则只写入到io_map_的最后一个元素。
     * 注意：写入数据长度+idx_start不要超过IO映射的长度。
     * @param idx_start 起始索引，从0开始，包含该索引。
     * @param data 写入的数据。
     */
    void writeIOMap(int idx_start, const std::vector<uint8_t> & data);

    /**
     * @brief 通过SDO读取寄存器中的数据。
     * @param slave 从站ID。
     * @param index 寄存器索引。
     * @param subindex 寄存器子索引。
     * @return 数据会拆分成uint8_t数组，数组的长度是实际读到的字节数。
     */
    std::vector<uint8_t> readSDO(int slave, int index, int subindex, int byte_size);

    /**
     * @brief 通过SDO写寄存器中的数据。
     * @param slave 从站ID。
     * @param index 寄存器索引。
     * @param subindex 寄存器子索引。
     * @param data 写入的数据，拆分成uint8_t数组。
     */
    void writeSDO(int slave, int index, int subindex, std::vector<uint8_t> data);
    uint8_t readInputManager(int slave_no, uint8_t channel) ;
    uint8_t readOutputManager(int slave_no, uint8_t channel) ;
    void writeManager(int slave_no, uint8_t channel, uint8_t value);
private:
    rclcpp::Logger logger_;
    std::mutex mutex_;
    std::string if_name_;   //!< 网口名称。
    uint8_t io_map_[4096];  //!< 用于映射数据的数组。
    int actual_map_size_;   //!< 实际数据长度，单位：Byte。
    int expected_wkc_;      
    bool is_working_;
    CustomConfigurationHook custom_config_hook_;
    std::map<std::string, EtherCATErrorCallback> registered_callbacks_;
};

} // namespace fieldbus

} // namespace baichuan

#endif // BAICHUAN_FIELDBUS_ETHERCAT_MASTER_H
