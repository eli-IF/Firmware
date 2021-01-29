/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "WorkItemExample.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

WorkItemExample::WorkItemExample() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1),
	_battery(1, this, 1000)
{
}

WorkItemExample::~WorkItemExample()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{
	_battery.updateBatteryStatus(
		hrt_absolute_time(),
		0.0,
		0.0,
		false,
		battery_status_s::BATTERY_SOURCE_POWER_MODULE,
		0,
		0.0
	);
	ScheduleOnInterval(1000_us); // 1000 us interval, 1000 Hz rate

	return true;
}

void
WorkItemExample::publishSmartBatteryStatusMsg(uint8_t smart_battery_buffer[])
{
  hrt_abstime tnow = hrt_absolute_time();

  // Extract battery parameters from message
  uint16_t pack_voltage_mv = getValueFromBatteryBuffer(smart_battery_buffer,4, 2);
  int16_t pack_current_da = getValueFromBatteryBuffer(smart_battery_buffer,6, 2);
  float bat_voltage_v = float(pack_voltage_mv)/1000.0F;

  float bat_current_a = float(pack_current_da)/-100.0F;
  //_actuators_sub.copy(&_actuator_controls);
  // _battery.updateBatteryStatus(tnow,
  // 			     bat_voltage_v,
  // 			     bat_current_a,
  // 			     true,
  // 			     battery_status_s::BATTERY_SOURCE_POWER_MODULE,
  // 			     0,
  // 			     _actuator_controls.control[actuator_controls_s::INDEX_THROTTLE]);

  // There is additional data that we get from the smart battery
  // Populate this here
  int16_t sku_code = getValueFromBatteryBuffer(smart_battery_buffer,2,0);
  int16_t pack_temp_c = getValueFromBatteryBuffer(smart_battery_buffer,8, 2);
  uint16_t remaining_capacity_pct = getValueFromBatteryBuffer(smart_battery_buffer,10, 2);
  uint16_t cycle_life_times = getValueFromBatteryBuffer(smart_battery_buffer,12, 2);
  // int16_t health_status_pct = getValueFromBatteryBuffer(smart_battery_buffer,14, 2);
  uint16_t cell_voltage_mv[12];
  uint16_t min_cell_voltage_mv = 65535;
  uint16_t max_cell_voltage_mv = 0;
  for (int i=0; i<12; i++) {
    cell_voltage_mv[i] = getValueFromBatteryBuffer(smart_battery_buffer,16+2*i, 2);
    // These values will be used later in the battery_status message
    min_cell_voltage_mv = (cell_voltage_mv[i] < min_cell_voltage_mv) ? cell_voltage_mv[i] : min_cell_voltage_mv;
    max_cell_voltage_mv = (cell_voltage_mv[i] > max_cell_voltage_mv) ? cell_voltage_mv[i] : max_cell_voltage_mv;
  }
  //uint16_t standard_capacity_mah = getValueFromBatteryBuffer(smart_battery_buffer,40, 2);
  uint16_t remaining_capacity_mah = getValueFromBatteryBuffer(smart_battery_buffer,42, 2);
  if (remaining_capacity_mah > max_cap) {
    	max_cap = remaining_capacity_mah;
  }
  uint32_t error_info = getValueFromBatteryBuffer(smart_battery_buffer,44, 2);
  _bat_status.id = 1;
  _bat_status.timestamp = tnow;
  _bat_status.connected = true;
  _bat_status.voltage_v = bat_voltage_v;
  _bat_status.voltage_filtered_v = _bat_status.voltage_v;
  _bat_status.current_a = bat_current_a;
  _bat_status.current_filtered_a = _bat_status.current_a;
  _bat_status.discharged_mah = float(max_cap - remaining_capacity_mah);
  _bat_status.remaining = float(remaining_capacity_pct/100.0F);
  _bat_status.temperature = float(pack_temp_c);
  _bat_status.cell_count = 12;
  _bat_status.capacity = remaining_capacity_mah;
  _bat_status.cycle_count = cycle_life_times;
  _bat_status.serial_number = sku_code;
  _bat_status.max_cell_voltage_delta = (max_cell_voltage_mv - min_cell_voltage_mv)/1000.0F;
  _bat_status.is_powering_off = false;

  // Determine battery warning level
  // Relevant battery error bits:
  // Low temp: 0 (1)
  // Over temp: 1 (2)
  // Pack under-voltage: 4 (16)
  // Pack over-voltage: 5 (32)
  // Cell voltage overspan: 6 (64)
  // Cell over-voltag1e: 7 (128)
  // Cell under-voltage: 8 (256)
  // Low remaining capacity: 11 (2048)
  if ((error_info & 16) || (error_info & 256) || (error_info & 2048)) {
    _bat_status.warning = 1;
  } else {
    _bat_status.warning = 0;
  }

  _bat_pub_topic.publish(_bat_status);
  //_bat_pub_topic.puclish(_bat_status_2)
}

int
WorkItemExample::getValueFromBatteryBuffer(uint8_t buffer[], int start_index, int num_bytes)
{
	unsigned value = 0;
	int multiplier = 1;
	while (num_bytes > 0) {
	  value += (buffer[start_index] * multiplier);
	  start_index++;
	  multiplier = multiplier << 8;
	  num_bytes--;
	}
	return value;
}

void WorkItemExample::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);


	// DO WORK

	if (_smart_battery_payload_sub.updated()) {
	    smart_battery_payload_s smart_battery_payload{};
	    _smart_battery_payload_sub.copy(&smart_battery_payload);
	    publishSmartBatteryStatusMsg(smart_battery_payload.payload);
  	}


	perf_end(_loop_perf);
}

int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int WorkItemExample::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[])
{
	return WorkItemExample::main(argc, argv);
}
