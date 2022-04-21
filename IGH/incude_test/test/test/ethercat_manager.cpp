#include <ethercat_manager.h>
// #include <ethercat_manager/ecat_dc.h>

#include <unistd.h>
#include <stdio.h>
// #include <time.h>

// #include <boost/ref.hpp>
// #include <boost/interprocess/sync/scoped_lock.hpp>

#include <ethercattype.h>
#include <ethercat_igh.h>


namespace
{
static const unsigned EC_TIMEOUTMON = 500;

static const RTIME period_xenomai = 900000;  /*900us */
static const RTIME timeout = 1000000; /*1 ms */
static bool is_stop = false;

RT_MUTEX mutex_;
RT_TASK task;

static void rt_check_domain_state(void)
{    
	ec_domain_state_t ds = {};	
	ecrt_domain_state(domain_0, &ds);    
	if (ds.working_counter != domain_state_0.working_counter) 
	{        
		printf("Domain1: WC %u.\n", ds.working_counter);    
	}    
	if (ds.wc_state != domain_state_0.wc_state) 
	{        
		printf("Domain1: State %u.\n", ds.wc_state);    
	}    
	domain_state_0 = ds;
}

static void rt_check_master_state(void)
{    
	ec_master_state_t ms;	
	ecrt_master_state(master, &ms);    
	if (ms.slaves_responding != master_state.slaves_responding) 
	{        
		rt_printf("%u slave(s).\n", ms.slaves_responding);    
	}    
	if (ms.al_states != master_state.al_states) 
	{        
		rt_printf("AL states: 0x%02X.\n", ms.al_states);    
	}    
	if (ms.link_up != master_state.link_up) 
	{        
		rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");    
	}    
	master_state = ms;
}

static void check_slave_config_states(int num)
{    
	ec_slave_config_state_t s;    
	ecrt_slave_config_state(sc_motor[num], &s);    
	
	if (s.al_state != sc_motor_state[num].al_state)        
		printf("sc_asda_state[0]: State 0x%02X.\n", s.al_state);    
	if (s.online != sc_motor_state[num].online)        
		printf("sc_asda_state[0]: %s.\n", s.online ? "online" : "offline");   
	if (s.operational != sc_motor_state[num].operational)        
		printf("sc_asda_state[0]: %soperational.\n",s.operational ? "" : "Not ");    
	sc_motor_state[num] = s;
}

void cycleWorker_xenomai(void *)
{
	static unsigned int cycle_counter = 0;
	rt_task_set_periodic(NULL, TM_NOW, period_xenomai);
	is_stop = false;
	printf("Func %s Line %d\n", __FUNCTION__, __LINE__);
	while(! is_stop)
	{
		rt_task_wait_period(NULL);
		ecrt_master_receive(master);    
		ecrt_domain_process(domain_0);
		rt_check_domain_state();    
		if (!(cycle_counter % 1000)) 
		{        
			rt_check_master_state();        
			check_slave_config_states(0);      
		}
		ecrt_domain_queue(domain_0);    
		ecrt_master_send(master);
		cycle_counter++;
	}
	printf("cycleWorker_xenomai stop!!!\n ");
}

} // end of anonymous namespace


namespace ethercat {

EtherCatManager::EtherCatManager(): num_clients_(0)
{
  fprintf(stderr, "Func %s Line %d\n", __FUNCTION__, __LINE__);	
  if (initIgh())
  {
	rt_mutex_create(&mutex_,"cyclerWorker_mutex");
	rt_task_create(&task, "cycleWorker_xenomai", 0, 99, 0);
	rt_task_start(&task, &cycleWorker_xenomai, (void *)NULL);	
  }
  else
 {
   // construction failed
   	throw fprintf(stderr, "Could not initialize SOEM");
 }
}

EtherCatManager::~EtherCatManager()
{
	is_stop = true;
	//stop SOEM, close socket
	rt_task_delete(&task);
	
	rt_mutex_delete(&mutex_);
	 if(master)    
	 {        
	 	printf("xenomai End of Program, release master\n");        
		ecrt_release_master(master);        
		master = NULL;    
	}
}

#define IF_ELMO(_ec_slave) 1
bool EtherCatManager::initIgh() {
	int ret;
	
	master = ecrt_request_master(0); // Index of the master to request.
	if (!master)
		return false;

	domain_0 = ecrt_master_create_domain(master);
  	if (!domain_0)
    		return false;

	 printf("xenomai Creating slave configurations...\n");    
	 sc_motor[0] = ecrt_master_slave_config(master, MotorSlavePos0, JMK_2DM522);    
	 if (!sc_motor[0]) 
	 	return false;
	 
	printf("xenomai configure fmmu and sm\n");  
	if (ecrt_slave_config_pdos(sc_motor[0], EC_END, slave_0_syncs)) 
		return false;
	
	printf("xenomai configure pdos\n");  
    	if (ecrt_domain_reg_pdo_entry_list(domain_0, domai_regs_0)) 
		return false;
	
	printf("xenomai Activating master...failed\n"); 
	if (ecrt_master_activate(master)) 
		return false;
	
 	printf("xenomai get domain data pointer\n");
	if (!(domain_pd_0 = ecrt_domain_data(domain_0))) 
	   	return false;
	printf("Func %s Line %d\n", __FUNCTION__, __LINE__);
	return true;
}


uint8_t EtherCatManager::writePDO_U8(int slave_no, uint16_t index, uint8_t subidx, uint8_t  value) const
{	
	int i;
	for (i = 0; i < DOMAI_REG_MAX; i++)
	{
		if (index == domai_regs_0[i].index &&
			subidx == domai_regs_0[i].subindex)
		{
			EC_WRITE_U8(*domai_regs_0[i].offset + domain_pd_0, value);
			break;
		}
	}
	
	return (i ==  DOMAI_REG_MAX);
}

uint8_t EtherCatManager::writePDO_U16(int slave_no, uint16_t index, uint8_t subidx, uint16_t  value) const
{	
	int i;
	for (i = 0; i < DOMAI_REG_MAX; i++)
	{
		if (index == domai_regs_0[i].index &&
			subidx == domai_regs_0[i].subindex)
		{
			EC_WRITE_U16(*domai_regs_0[i].offset + domain_pd_0, value);
			break;
		}
	}
	
	return (i ==  DOMAI_REG_MAX);
}

uint8_t EtherCatManager::writePDO_U32(int slave_no, uint16_t index, uint8_t subidx, uint32_t  value) const
{	
	int i;
	for (i = 0; i < DOMAI_REG_MAX; i++)
	{
		if (index == domai_regs_0[i].index &&
			subidx == domai_regs_0[i].subindex)
		{
			EC_WRITE_U32(*domai_regs_0[i].offset + domain_pd_0, value);
			break;
		}
	}
	
	return (i ==  DOMAI_REG_MAX);
}

uint8_t EtherCatManager::readPDO_U8(int slave_no, uint16_t index, uint8_t subidx) const
{
	int i;
	for (i = 0; i < DOMAI_REG_MAX; i++)
	{
		if (index == domai_regs_0[i].index &&
			subidx == domai_regs_0[i].subindex)
		{
			return EC_READ_U8(*domai_regs_0[i].offset + domain_pd_0);
		}
	}
	
	return -1;
}

uint16_t EtherCatManager::readPDO_U16(int slave_no, uint16_t index, uint8_t subidx) const
{
	int i;
	for (i = 0; i < DOMAI_REG_MAX; i++)
	{
		if (index == domai_regs_0[i].index &&
			subidx == domai_regs_0[i].subindex)
		{
			return EC_READ_U16(*domai_regs_0[i].offset + domain_pd_0);
		}
	}
	
	return -1;
}

uint32_t EtherCatManager::readPDO_U32(int slave_no, uint16_t index, uint8_t subidx) const
{
	int i;
	for (i = 0; i < DOMAI_REG_MAX; i++)
	{
		if (index == domai_regs_0[i].index &&
			subidx == domai_regs_0[i].subindex)
		{
			return EC_READ_U32(*domai_regs_0[i].offset + domain_pd_0);
		}
	}
	
	return -1;
}


int EtherCatManager::getNumClinets() const
{
	return num_clients_;
}

}


