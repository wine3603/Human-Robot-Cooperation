#include "elfin_ethercat_driver/elfin_ethercat_manager_zombie.h"

#include <unistd.h>
#include <stdio.h>
#include <time.h>

#include <boost/ref.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "soem/ethercat.h"


namespace 
{


} // end of anonymous namespace


namespace elfin_ethercat_driver {

EtherCatManagerZombie::EtherCatManagerZombie(const std::string& ifname)
  : EtherCatManager(ifname,true),ifname_(ifname),
    num_clients_(0),
    stop_flag_(false)
{
  // initialize iomap
  for(int i=0; i<4096; i++)
  {
    iomap_[i]=0;
  }

  if (initSoem(ifname)) 
  {
      // cycle_thread_ = boost::thread(boost::bind(&EtherCatManagerZombie::cycleWorker, 
      // this, boost::ref(iomap_mutex_), 
      //   boost::ref(stop_flag_)));
  } 
  else 
  {
    // construction failed
    throw EtherCatError("Could not initialize SOEM");
  }
}

EtherCatManagerZombie::~EtherCatManagerZombie()
{
  stop_flag_ = true;

  // Request init operational state for all slaves
  ec_slave[0].state = EC_STATE_INIT;

  /* request init state for all slaves */
  ec_writestate(0);

  //stop SOEM, close socket
  ec_close();
  cycle_thread_.join();
}
int EtherCatManagerZombie::customCofiguration(uint16_t slave)
{
    uint16_t map_1c12[3] = {0x0001, 0x1600};
    uint16_t map_1c13[3] = {0x0001, 0x1A00};
    ec_SDOwrite(slave, 0x1C12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x1C13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

    return 0;
}
bool EtherCatManagerZombie::initSoem(const std::string& ifname) {
  // Copy string contents because SOEM library doesn't 
  // practice const correctness
  const static unsigned MAX_BUFF_SIZE = 1024;
  char buffer[MAX_BUFF_SIZE];
  size_t name_size = ifname_.size();
  if (name_size > sizeof(buffer) - 1) 
  {
    fprintf(stderr, "Ifname %s exceeds maximum size of %u bytes\n", ifname_.c_str(), MAX_BUFF_SIZE);
    return false;
  }
  std::strncpy(buffer, ifname_.c_str(), MAX_BUFF_SIZE);

  printf("Initializing etherCAT master\n");

  // if (!ec_init(buffer))
  // {
  //   fprintf(stderr, "Could not initialize ethercat driver\n");
  //   return false;
  // }

  /* find and auto-config slaves */
  // if (ec_config_init(FALSE) <= 0)
  // {
  //   fprintf(stderr, "No slaves are found on %s\n", ifname_.c_str());
  //   return false;
  // }

  printf("SOEM found and configured %d slaves\n", ec_slavecount);

  // if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_PRE_OP)
  // {
  //   fprintf(stderr, "Could not set EC_STATE_PRE_OP\n");
  //   return false;
  // }


  // configure IOMap
  // int iomap_size = ec_config_map(iomap_);
  // printf("SOEM IOMap size: %d\n", iomap_size);

  // locates dc slaves - ???
  // ec_configdc();

  // '0' here addresses all slaves
  // if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP)
  // {
  //   fprintf(stderr, "Could not set EC_STATE_SAFE_OP\n");
  //   return false;
  // }

  /* 
      This section attempts to bring all slaves to operational status. It does so
      by attempting to set the status of all slaves (ec_slave[0]) to operational,
      then proceeding through 40 send/recieve cycles each waiting up to 50 ms for a
      response about the status. 
  */
  //   ec_slave[1].state = EC_STATE_OPERATIONAL;
  //   ec_send_processdata();
  //   ec_receive_processdata(EC_TIMEOUTRET);

  //   ec_writestate(1);
  //   int chk = 40;
  //   do {
  //     ec_send_processdata();
  //     ec_receive_processdata(EC_TIMEOUTRET);
  //     ec_statecheck(1, EC_STATE_OPERATIONAL, 50000); // 50 ms wait for state check
  //   } while (chk-- && (ec_slave[1].state != EC_STATE_OPERATIONAL));

  //   if(ec_statecheck(1,EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
  //   {
  //     fprintf(stderr, "OPERATIONAL state not set, exiting\n");
  //     return false;
  //   }

  // ec_readstate();

  printf("\nFinished configuration successfully\n");
  return true;
}

int EtherCatManagerZombie::getNumClinets() const
{
  return num_clients_;
}

void EtherCatManagerZombie::write(int slave_no, uint8_t channel, uint8_t value)
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if (slave_no > ec_slavecount) {
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Obits) {
    fprintf(stderr, "ERROR : slave_no(%d) : channel(%d) is larger than Output bits (%d), you may need to read elfin_robot/docs/Fix_ESI.md or elfin_robot/docs/Fix_ESI_english.md with a Markdown editor or on github.com\n", slave_no, channel*8, ec_slave[slave_no].Obits);
    exit(1);
  }
  ec_slave[slave_no].outputs[channel] = value;
}

uint8_t EtherCatManagerZombie::readInput(int slave_no, uint8_t channel) const
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if (slave_no > ec_slavecount) {
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Ibits) {
    fprintf(stderr, "ERROR : slave_no(%d) : channel(%d) is larger than Input bits (%d), you may need to read elfin_robot/docs/Fix_ESI.md or elfin_robot/docs/Fix_ESI_english.md with a Markdown editor or on github.com\n", slave_no, channel*8, ec_slave[slave_no].Ibits);
    exit(1);
  }
  return ec_slave[slave_no].inputs[channel];
}

uint8_t EtherCatManagerZombie::readOutput(int slave_no, uint8_t channel) const
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if (slave_no > ec_slavecount) {
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Obits) {
    fprintf(stderr, "ERROR : slave_no(%d) : channel(%d) is larger than Output bits (%d), you may need to read elfin_robot/docs/Fix_ESI.md or elfin_robot/docs/Fix_ESI_english.md with a Markdown editor or on github.com\n", slave_no, channel*8, ec_slave[slave_no].Obits);
    exit(1);
  }
  return ec_slave[slave_no].outputs[channel];
}

template <typename T>
uint8_t EtherCatManagerZombie::writeSDOImpl(int slave_no, uint16_t index, uint8_t subidx, T value) const
{
  int ret;
  ret = ec_SDOwrite(slave_no, index, subidx, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE);
  return ret;
}
void EtherCatManagerZombie::writeSDO(int slave, int index, int subindex, std::vector<uint8_t> data)
{
    int byte_size = data.size();
    ec_SDOwrite(slave, index, subindex, FALSE, byte_size, data.data(), EC_TIMEOUTSAFE);
}

template <typename T>
T EtherCatManagerZombie::readSDOImpl(int slave_no, uint16_t index, uint8_t subidx) const
{
  int ret, l;
  T val;
  l = sizeof(val);
  ret = ec_SDOread(slave_no, index, subidx, FALSE, &l, &val, EC_TIMEOUTRXM);
  if ( ret <= 0 ) { // ret = Workcounter from last slave response
    fprintf(stderr, "Failed to read from ret:%d, slave_no:%d, index:0x%04x, subidx:0x%02x\n", ret, slave_no, index, subidx);
  }
  return val;
}
std::vector<uint8_t> EtherCatManagerZombie::readSDO(int slave, int index, int subindex, int byte_size)
{
    int size = byte_size;   // 实际读到的字节数会返回。
    std::vector<uint8_t> data(size);
    ec_SDOread(slave, index, subindex, FALSE, &size, data.data(), EC_TIMEOUTSAFE);
    std::vector<uint8_t> ret(size);
    for (int idx = 0; idx < size; ++idx)
        ret[idx] = data[idx];

    return ret;
}
static const unsigned THREAD_SLEEP_TIME = 1000; // 1 ms
static const unsigned EC_TIMEOUTMON = 500;
static const int NSEC_PER_SECOND = 1e+9;
void EtherCatManagerZombie::timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
      tick.tv_nsec -= NSEC_PER_SECOND;
      tick.tv_sec++;
    }
}

void EtherCatManagerZombie::handleErrors()
{
  /* one ore more slaves are not responding */
  ec_group[0].docheckstate = FALSE;
  ec_readstate();
  for (int slave = 1; slave <= ec_slavecount; slave++)
  {
    if ((ec_slave[slave].group == 0) && (ec_slave[slave].state != EC_STATE_OPERATIONAL) )
    {
      ec_group[0].docheckstate = TRUE;
      if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
      {
        fprintf(stderr, "ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
        ec_writestate(slave);
      }
      else if(ec_slave[slave].state == EC_STATE_SAFE_OP )
      {
        fprintf(stderr, "WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
        ec_slave[slave].state = EC_STATE_OPERATIONAL;
        ec_writestate(slave);
      }
      else if(ec_slave[slave].state > 0)
      {
        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
        {
          ec_slave[slave].islost = FALSE;
          printf("MESSAGE : slave %d reconfigured\n",slave);
        }
      }
      else if(!ec_slave[slave].islost)
      {
        /* re-check state */
        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
        if (!ec_slave[slave].state)
        {
          ec_slave[slave].islost = TRUE;
          fprintf(stderr, "ERROR : slave %d lost\n",slave);
        }
      }
    }
    if (ec_slave[slave].islost)
    {
      if(!ec_slave[slave].state)
      {
        if (ec_recover_slave(slave, EC_TIMEOUTMON))
        {
          ec_slave[slave].islost = FALSE;
          printf("MESSAGE : slave %d recovered\n",slave);
        }
      }
      else
      {
        ec_slave[slave].islost = FALSE;
        printf("MESSAGE : slave %d found\n",slave);
      }
    }
  }
}


void EtherCatManagerZombie::cycleWorker(boost::mutex& mutex, bool& stop_flag)
{
  // 1ms in nanoseconds
  double period = THREAD_SLEEP_TIME * 1000;
  // get current time
  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  timespecInc(tick, period);
  // time for checking overrun
  struct timespec before;
  double overrun_time;
  while (!stop_flag) 
  {
    int expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    int sent, wkc;
    {
      boost::mutex::scoped_lock lock(mutex);
      sent = ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
    }

    if (wkc < expected_wkc)
    {
      handleErrors();
    }

    // check overrun
    clock_gettime(CLOCK_REALTIME, &before);
    overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -  (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
    if (overrun_time > 0.0)
    {
      tick.tv_sec=before.tv_sec;
      tick.tv_nsec=before.tv_nsec;
    }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    timespecInc(tick, period);
  }
}

}

