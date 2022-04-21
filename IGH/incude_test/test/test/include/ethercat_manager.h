#ifndef ETHERCAT_MANAGER_H
#define ETHERCAT_MANAGER_H

#include<inttypes.h> 
#include<stdint.h>
#include<cstdint>

#include<iostream>
//#include "common.h"
typedef enum {  NOT_READY = 0, 
                SWITCH_DISABLED,
                READY_SWITCH, 
                SWITCHED_ON,
                OPERATION_ENABLED,
                QUICK_STOP,
                FAULT_REACTION,
                FAULT,
                UNKNOWN} PDS_STATUS;  // Statusword(6041h) 

typedef enum {  NO_MODE_CHANGE_1 = 0,
                PROFILE_POSITION_MODE,
                VELOCITY_MODE,
                PROFILE_VELOCITY_MODE,
                TORQUE_PROFILE_MODE,
                NO_MODE_CHANGE_2,
                HOMING_MODE,
                INTERPOLATED_POSITION_MODE,
                CYCLIC_SYNCHRONOUS_POSITION_MODE,
                CYCLIC_SYNCHRONOUS_VELOCITY_MODE,
                CYCLIC_SYNCHRONOUS_TORQUE_MODE} PDS_OPERATION; // Mode of operation(6061h) 
#define DEFAULT_INTERPOLATION_TIME_PERIOD       1000000 /* 1ms */


namespace ethercat {
#if 0
/**
 * \brief EtherCAT exception. Currently this is only thrown in the event
 *        of a failure to construct an EtherCat manager.
 */
class EtherCatError : public std::runtime_error
{
public:
    explicit EtherCatError(const std::string& what)
    : std::runtime_error(what)
    {}
};
#endif

    /**
    * \brief This class provides a CPP interface to the SimpleOpenEthercatMaster library
    * Given the name of an ethernet device, such as "eth0", it will connect,
    * start a thread that cycles data around the network, and provide read/write
    * access to the underlying io map.
    *
    * Please note that as used in these docs, 'Input' and 'Output' are relative to
    * your program. So the 'Output' registers are the ones you write to, for example.
    */
class EtherCatManager
{
    
public:
    /**
    * \brief Constructs and initializes the ethercat slaves on a given network interface.
    *
    * @param[in] ifname the name of the network interface that the ethercat chain 
    *                   is connected to (i.e. "eth0")
    *
    * Constructor can throw EtherCatError exception if SOEM could not be 
    * initialized.
    */
    EtherCatManager();


    ~EtherCatManager();

    /**
    * \brief writes 'value' to the 'channel-th' output-register of the given 'slave'
    *  
    * @param[in] slave_no The slave number of the device to write to (>= 1)
    * @param[in] channel The byte offset into the output IOMap to write value to
    * @param[in] value The byte value to write
    *
    * This method currently makes no attempt to catch out of bounds errors. Make
    * sure you know your IOMap bounds.
    */
    void write(int slave_no, uint8_t channel, uint8_t value);
	
    /**
    * \brief write the SDO object of the given slave no
    *
    * @param[in] slave_no The slave number of the device to read from (>= 1)
    * @param[in] index The index address of the parameter in SDO object
    * @param[in] subidx The sub-index address of the parameter in SDO object
    * @param[in] value value to write
    */
    uint8_t writePDO_U8(int slave_no, uint16_t index, uint8_t subidx, uint8_t value) const;
    uint8_t writePDO_U16(int slave_no, uint16_t index, uint8_t subidx, uint16_t value) const;
    uint8_t writePDO_U32(int slave_no, uint16_t index, uint8_t subidx, uint32_t value) const;	 
    /**
    * \brief read the SDO object of the given slave no
    *
    * @param[in] slave_no The slave number of the device to read from (>= 1)
    * @param[in] index The index address of the parameter in SDO object
    * @param[in] subidx The sub-index address of the parameter in SDO object
    */
    uint8_t readPDO_U8(int slave_no, uint16_t index, uint8_t subidx) const;
    uint16_t readPDO_U16(int slave_no, uint16_t index, uint8_t subidx) const;
    uint32_t readPDO_U32(int slave_no, uint16_t index, uint8_t subidx) const;	
    /**
    * \brief get the number of clients
    */
    int getNumClinets() const;

    // /**
    // * \brief init the DC time
    // */ 
    // void initDistributeClock();

    // /**
    // * \brief wait for next period.
    // */ 
    // void waitnextperiod();

    // /**
    // * \brief update the DC time
    // */ 
    // void updateDistributeClock();

    /*
        brief : 
     */
    //bool ifRealtime() const;


    // void sent_receive_eth_msg();

private:

    bool initIgh();
	
    //uint8_t iomap_[4096];
    int num_clients_;
    //mutable boost::mutex iomap_mutex_;
    int wkc;

};

}

#endif