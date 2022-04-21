#include <stdio.h>

#include <elmo_client.h>
#include <ethercat_igh.h>

namespace elmo_control
{

ElmoClient::ElmoClient(ethercat::EtherCatManager& manager, int slave_no)
  : manager_(manager)
  , slave_no_(slave_no)
{}

ElmoClient::~ElmoClient(){
}


void ElmoClient::writeOutputs(const ElmoOutput& output)
{

}

ElmoInput ElmoClient::readInputs() const
{ 
	ElmoInput input;
	return input;
}

ElmoOutput ElmoClient::readOutputs() const
{

	ElmoOutput output;
	return output;
}

void ElmoClient::reset()
{
	ElmoOutput output;
	memset(&output, 0x00, sizeof(ElmoOutput));
	output.controlword = 0x0080; // fault reset
	writeOutputs(output);
	printf("Fault was cleared\n");
}

void ElmoClient::servoOn()
{
	ElmoInput input = readInputs();
	printPDSStatus(input);
	ElmoOutput output;
	memset(&output, 0x00, sizeof(ElmoOutput));
	int loop = 0;
	while (getPDSStatus(input) != OPERATION_ENABLED) {
		switch ( getPDSStatus(input) ) {
			case SWITCH_DISABLED:
				output.controlword = 0x0006; // move to ready to switch on
				break;
			case READY_SWITCH:
				output.controlword = 0x0007; // move to switched on
				break;
			case SWITCHED_ON:
				output.controlword = 0x000f; // move to operation enabled
				break;
			case OPERATION_ENABLED:
				break;
			default:
				// printf("unknown status");
				output.controlword &= ~(1 << 7); 
				writeOutputs(output);
				output.controlword |= (1 << 7); 
				writeOutputs(output);
				break;
				// return;
		}
		writeOutputs(output);
		//rt_timer_spin(DEFAULT_INTERPOLATION_TIME_PERIOD);
		input = readInputs();
		if (loop++ % 1000 == 1) printPDSStatus(input);
	}
}

void ElmoClient::servoOff()
{
	ElmoInput input = readInputs();
	printPDSStatus(input);
	ElmoOutput output;
	memset(&output, 0x00, sizeof(ElmoOutput));
	int loop = 0;
	while (getPDSStatus(input) != SWITCH_DISABLED) {
		switch ( getPDSStatus(input) ) {
			case READY_SWITCH:
				output.controlword = 0x0000; // disable voltage
				break;
			case SWITCHED_ON:
				output.controlword = 0x0006; // shutdown
				break;
			case OPERATION_ENABLED:
				output.controlword = 0x0007; // disable operation
				break;
			default:
				// printf("unknown status");
				output.controlword = 0x0000; // disable operation
				break;
		}
		writeOutputs(output);
		//rt_timer_spin(DEFAULT_INTERPOLATION_TIME_PERIOD);
		input = readInputs();
		if (loop++ % 100 == 1) printPDSStatus(input);
	}
}

void ElmoClient::shutDown()
{
	ElmoOutput output;
	output.controlword = 0x0006; // shutdown
	writeOutputs(output);
}

// void ElmoClient::quickStop()
// {
// 	output.controlword &= 0x0002; // quick stop 
// 	writeOutputs(output);
// }

PDS_STATUS ElmoClient::getPDSStatus(const ElmoInput input) const
{
	uint16 statusword = input.statusword;
	// printf("statusword: %4x \n",statusword );
	if (((statusword) & 0x004f) == 0x0000) { // x0xx 0000
		return NOT_READY;
	}else if (((statusword) & 0x004f) == 0x0040) { // x1xx 0000
		return SWITCH_DISABLED;
	}else if (((statusword) & 0x006f) == 0x0021) { // x01x 0001
		return READY_SWITCH;
	}else if (((statusword) & 0x006f) == 0x0023) { // x01x 0011
		return SWITCHED_ON;
	}else if (((statusword) & 0x006f) == 0x0027) { // x01x 0111
		return OPERATION_ENABLED;
	}else if (((statusword) & 0x006f) == 0x0007) { // x00x 0111
		return QUICK_STOP;
	}else if (((statusword) & 0x004f) == 0x000f) { // x0xx 1111
		return FAULT_REACTION;
	}else if (((statusword) & 0x004f) == 0x0008) { // x0xx 1000
		return FAULT;
	}else{
		return UNKNOWN;
  }
}

PDS_OPERATION ElmoClient::getPDSOperation(const ElmoInput input) const
{
	int8 operation_mode = input.operation_mode;
	switch (operation_mode) {
	case 0: return NO_MODE_CHANGE_1;	break;
	case 1: return PROFILE_POSITION_MODE;	break; // pp
	case 2: return VELOCITY_MODE;		break; // vl
	case 3: return PROFILE_VELOCITY_MODE;	break; // pv
	case 4: return TORQUE_PROFILE_MODE;	break; // tq
	case 5: return NO_MODE_CHANGE_2;		break; // hm
	case 6: return HOMING_MODE;		break; // hm
	case 7: return INTERPOLATED_POSITION_MODE;	break; // ip
	case 8: return CYCLIC_SYNCHRONOUS_POSITION_MODE;	break; // csp
	case 9: return CYCLIC_SYNCHRONOUS_VELOCITY_MODE;	break; // csv
	case 10: return CYCLIC_SYNCHRONOUS_TORQUE_MODE;	break; // cst
	}
}

PDS_STATUS ElmoClient::getPDSControl(const ElmoInput input) const
{
	// uint16 statusword = input.statusword;
}



void ElmoClient::printPDSStatus(const ElmoInput input) const
{
	printf("Statusword(6041h): %04x\n ", input.statusword);
	switch ( getPDSStatus(input) ) {
	case NOT_READY:
		printf("Not ready to switch on\n");
		break;
	case SWITCH_DISABLED:
		printf("Switch on disabled\n");
		break;
	case READY_SWITCH:
		printf("Ready to switch on\n");
		break;
	case SWITCHED_ON:
		printf("Switched on\n");
		break;
	case OPERATION_ENABLED:
		printf("Operation enabled\n");
		break;
	case QUICK_STOP:
		printf("Quick stop active\n");
		break;
	case FAULT_REACTION:
		printf("Fault reaction active\n");
		break;
	case FAULT:
		printf("Fault\n");
		break;
	case UNKNOWN:
		printf("Unknown status %04x\n", input.statusword);
		break;
	}
	if ( input.statusword & 0x0800 ) {
		printf(" Internal limit active\n");
	}


	switch ( getPDSOperation(input) ) {
		case PROFILE_POSITION_MODE:
			if ( (input.statusword & 0x3400) | 0x2000 ) {
				printf(" Following error\n");
			}
			if ( (input.statusword & 0x3400) | 0x1000 ) {
				printf(" Set-point acknowledge\n");
			}
			if ( (input.statusword & 0x3400) | 0x0400 ) {
				printf(" Target reached\n");
			}
			break;
		case VELOCITY_MODE:
			break;
		case PROFILE_VELOCITY_MODE:
			if ( (input.statusword & 0x3400) | 0x2000 ) {
				printf(" Max slippage error (Not supported)\n");
			}
			if ( (input.statusword & 0x3400) | 0x1000 ) {
				printf(" Speed\n");
			}
			if ( (input.statusword & 0x3400) | 0x0400 ) {
				printf(" Target reached\n");
			}
			break;
		case TORQUE_PROFILE_MODE:
			if ( (input.statusword & 0x3400) | 0x0400 ) {
				printf(" Target reached\n");
			}
			break;
		case HOMING_MODE:
			if ( (input.statusword & 0x3400) | 0x2000 ) {
				printf(" Homing error\n");
			}
			if ( (input.statusword & 0x3400) | 0x1000 ) {
				printf(" Homing attained\n");
			}
			if ( (input.statusword & 0x3400) | 0x0400 ) {
				printf(" Target reached\n");
			}
			break;
		case INTERPOLATED_POSITION_MODE:
			if ( (input.statusword & 0x3400) | 0x1000 ) {
				printf(" Ip mode active\n");
			}
			if ( (input.statusword & 0x3400) | 0x0400 ) {
				printf(" Target reached\n");
			}
			break;
		case CYCLIC_SYNCHRONOUS_POSITION_MODE:
			if ( (input.statusword & 0x3400) | 0x2000 ) {
				printf(" Following error\n");
			}
			if ( (input.statusword & 0x3400) | 0x1000 ) {
				printf(" Drive follows command value\n");
			}
			break;
		case CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
			if ( (input.statusword & 0x3400) | 0x1000 ) {
				printf(" Drive follows command value\n");
			}
			break;
			case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
			if ( (input.statusword & 0x3400) | 0x1000 ) {
				printf(" Drive follows command value\n");
			}
			break;
	}
}

void ElmoClient::printPDSOperation(const ElmoInput input) const
{
	printf("Mode of operation(6061h): %04x\n ", input.operation_mode);
	switch ( getPDSOperation(input) ) {
		case NO_MODE_CHANGE_1:
			printf("No mode change / no mode assigned\n");
			break;
		case PROFILE_POSITION_MODE:
			printf("Profile position mode\n");
			break;
		case VELOCITY_MODE:
			printf("Velocity mode\n");
			break;
		case PROFILE_VELOCITY_MODE:
			printf("Profile velocity mode\n");
			break;
		case TORQUE_PROFILE_MODE:
			printf("Torque profile mode\n");
			break;
		case NO_MODE_CHANGE_2:
			printf("No mode change / no mode assigned\n");
			break;
		case HOMING_MODE:
			printf("Homing mode\n");
			break;
		case INTERPOLATED_POSITION_MODE:
			printf("Interpolated position mode\n");
			break;
		case CYCLIC_SYNCHRONOUS_POSITION_MODE:
			printf("Cyclic synchronous position mode\n");
			break;
		case CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
			printf("Cyclic synchronous velocity mode\n");
			break;
		case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
			printf("Cyclic synchronous torque mode\n");
			break;
		default:
			printf("Reserved %04x\n", input.operation_mode);
			break;
	}
}

void ElmoClient::printPDSControl(const ElmoInput input) const
{
	
}

  int ElmoClient::sentPosDemandValue(int32_t i32val){
	  //	return (setSDO( uint16_t(0x6062), i32val));

  }


int ElmoClient::setProfileVelocity(uint32_t val)
{
	printf("setProfileVelocity...\n");

	//return (setSDO( uint16_t(0x6081), val));

}

int ElmoClient::setMaxProfileVelocity(uint32_t val){
	printf("setMaxProfileVelocity...\n");
	//return (setSDO( uint16_t(0x607f), val));
}


int ElmoClient::setMaxVelocity(uint32_t max_velocity)
{
	printf("setMaxVelocity...\n");
	//return (setSDO( uint16_t(0x6080), max_velocity));
}

int ElmoClient::setMotorRateTorque(uint32_t rate_torque)
{
	printf("setMotorRateTorque...\n");
	//return (setSDO( uint16_t(0x6076), rate_torque));
}

int ElmoClient::setMaxTorque(uint16_t max_torque){

	printf("setMaxTorque...\n");
	//return (setSDO( uint16_t(0x6072), max_torque));

}

int ElmoClient::setRateCurrent(uint32_t rate_cur){
	//return (setSDO( uint16_t(0x6075) ,rate_cur));
}

int ElmoClient::setMaxCurrent(uint16_t max_cur){
	//return (setSDO( uint16_t(0x6073), max_cur));
	  
}


int ElmoClient::setPosRangeLimit(uint32_t down_range,uint32_t up_range){

	int ret = 0;
	// ret += setSDO( uint16_t(0x607b), 2, uint8_t(0x00));
	//ret += setSDO( uint16_t(0x607b), down_range, uint8_t(0x01));
	//ret += setSDO( uint16_t(0x607b), up_range, uint8_t(0x02));
	return ret;
}

int ElmoClient::setPosOptionCode(uint16_t val){
	//return (setSDO(uint16_t(0x60F2) ,val));
}

uint16_t ElmoClient::readPosOptionCode(){
	//return  (manager_.readSDO<uint16_t>(slave_no_,0x60f2,0x00));
}

uint32_t ElmoClient::readMotorRateTorque()const
{
    //return (manager_.readSDO<uint32_t>(slave_no_,0x6076,0x00));
}

uint32_t ElmoClient::getAbsolutePos()const{
    //return (manager_.readSDO<uint32_t>(slave_no_,0x3091,0x02));
}

uint32_t ElmoClient::getAbsoluteVel()const{
	//return (manager_.readSDO<uint32_t>(slave_no_,0x3097,0x02));
}

void ElmoClient::setInterpolationTimePeriod(int us)
{
	uint32_t u32val;	/*time unit : ns */
	uint8_t u8val;
	switch ( us ) {
		case 1000: u32val = 1000000; 	u8val = 1; break;
		case 2000: u32val = 2000000; 	u8val = 2; break;
		case 4000: u32val = 4000000; 	u8val = 4; break;
		default:
			fprintf(stderr, "setInterpolatinTimePeriod(%d) must be ether of 1000, 2000, 4000\n", us);
		return;
	}
	interpolationTimePeriod = u8val;
	int ret = 0;
	//ret += manager_.writeSDO<uint32_t>(slave_no_, 0x1c32, 0x02, u32val);
	//ret += manager_.writeSDO<uint8_t>(slave_no_, 0x60c2, 0x01, u8val);
	printf("Set interpolation time period %d us\n", us);

	//u32val = manager_.readSDO<uint32_t>(slave_no_, 0x1c32, 0x02);
	//u8val = manager_.readSDO<uint8_t>(slave_no_, 0x60c2, 0x01);
	printf("1c32h: cycle time %04x\n", u32val);
	printf("60c2h: interpolation time period value %02d\n", u8val);

}

int ElmoClient::getInterpolationTimePeriod(){
	return interpolationTimePeriod;
}


uint32_t ElmoClient::readTrajBuffSize() const
{
	//return (manager_.readSDO<uint32_t>(slave_no_, 0x60c4, 0x2));
}

uint16_t ElmoClient::readActualBuffPos() const
{
	//return (manager_.readSDO<uint16_t>(slave_no_, 0x60c4, 0x4));
}

  int ElmoClient::setAcc(uint32_t u32val){

	printf("setAcc...\n");
	//return (setSDO( uint16_t(0x6083), u32val));
  }

  int ElmoClient::setDcc(uint32_t u32val){

	printf("setDcc...\n");
//	return (setSDO( uint16_t(0x6084), u32val));
  }

  int ElmoClient::setMaxAcc(int32_t i32val){
	printf("setMaxAcc...\n");
	//return (setSDO( uint16_t(0x60c5), i32val));
  }

  int ElmoClient::setMaxDcc(int32_t i32val){
	printf("setMaxDcc...\n");
	//return (setSDO( uint16_t(0x60c6), i32val));
  }

  int16_t ElmoClient::readChipTemp(){
	//return (manager_.readSDO<uint16_t>(slave_no_, 0x22a3, 0x1));
  }

template <class DATA_TYPE >
int ElmoClient::setSDO( uint16_t index, DATA_TYPE val, uint8_t sub_index)
{
	int ret = 0 ;
#if 0
    manager_.writeSDO<DATA_TYPE>(slave_no_, index, sub_index, val);

 
    DATA_TYPE temp;
    int timeout = 10;
    RTIME now, previous;	/*ns */
    previous = rt_timer_read(); 

    while(temp != val){

		manager_.writeSDO<DATA_TYPE>(slave_no_, index, sub_index, val);
		temp = manager_.readSDO<DATA_TYPE>(slave_no_,index,sub_index);
		rt_timer_spin(DEFAULT_INTERPOLATION_TIME_PERIOD); 

		now = rt_timer_read(); 

		if((now - previous) / 1e9 >= timeout ){	
			ret = -1;
			break;
      	} 
    }
#endif
    return ret;
}

} // end of elmo_control namespace