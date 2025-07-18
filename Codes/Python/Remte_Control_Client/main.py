# Requires the following python libraries:
# numpy nicegui pywebview bleak

import threading
import queue
import json
from json.decoder import JSONDecodeError
from nicegui import app, ui, native
import asyncio
import math
import numpy as np
from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
import logging
import time
import struct

logger = logging.getLogger(__name__)

iq = queue.SimpleQueue()
oq = queue.SimpleQueue()
tq = queue.SimpleQueue()
dq = queue.SimpleQueue()

DEFAULT_CAR_NAME = 'RClub_Car'
TELEMETRY_LENGTH = 230
VERSION_STR = '2.22'

glob_model = {}
glob_model['is_init'] = False
glob_model['is_ui_init'] = False
glob_model['is_connected_chip'] = None
glob_model['is_disconnected_chip'] = None
glob_model['joystick'] = [0,0,0]
glob_model['heading_joystic_angle'] = 0
glob_model['joystick_request_speed'] = [0,0]
glob_model['telemetry_str_len'] = 0
glob_model['last_telemetry_time'] = 0
glob_model['cycles_in_telemetry'] = 0
glob_model['mcu_times_since_telemetry'] = 0
glob_init_semaphore = threading.Semaphore()
glob_UI_disconnected = 0
glob_BLE_connected = 0

class DeviceNotFoundError(Exception):
    pass

async def ble_task(input_queue, output_queue, telemetry_Queue, data_queue):
    target_name = glob_model['CAR_NAME']
    global glob_BLE_connected
    glob_BLE_connected = 1
    devices = await BleakScanner.discover()
    target_found = False
    for d in devices:
        print(d)
        if d.name == target_name:
            target_found = True
            break
    target_device = d

    if target_found:
        async with BleakClient(target_device) as client:

            async def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
                """Simple notification handler which prints the data received."""
                # logger.info("%s: %r", characteristic.uuid, data.decode())

                if (characteristic.uuid == "19B10001-E8F2-537E-4F6C-D104768A1214".lower()):
                    
                    # logger.info("%s: %r", "Telemetry notify", data[0:80].hex(' '))
                    
                    # We first decode the notification packet

                    # The struct format string corresponds to the updated telemetry packet structure
                    # see ble_telemetry_avail_data in BLE_Comm.hpp
                    struct_format = "<BBIhhh16x"

                    # Unpack the fixed part of the packet
                    try:
                        telemetry_avail_flag, n_tracker_points, sys_time, left_speed, right_speed, heading = struct.unpack_from(struct_format, data)
                        heading = (float)(heading) / 10.0  # Convert heading from 0.1 deg to degrees
                        # logger.info("Telemetry Flag: %d, Tracker Points: %d, Sys Time: %d, Left Speed: %d, Right Speed: %d, heading: %d",
                        #             telemetry_avail_flag, n_tracker_points, sys_time, left_speed, right_speed, heading)
                    except struct.error as e:
                        logger.error("Error unpacking telemetry packet: %s", e)
                        return
                    
                    # Decode tracker data from the array part of the struct
                    tracker_data = []
                    # Updated tracker_data_format for new track_point struct
                    tracker_data_format = "<IhhBB2xhhhh"  # heading_and_distance, lin_accel, gyro, echo_range_cm, status_flags, 2 spare, 8 spare
                    tracker_data_size = struct.calcsize(tracker_data_format)
                    fixed_part_size = struct.calcsize(struct_format)  # Size of the fixed part of the struct

                    for i in range(n_tracker_points):
                        offset = fixed_part_size + i * tracker_data_size  # Fixed part is 20 bytes, then tracker points follow
                        try:
                            _xy, _linaccel, _gyro, echo, status, _i, _o, _os, _e = struct.unpack_from(tracker_data_format, data, offset)
                            # x position is stored as the upper 16 bits of the first integer, y position is stored as the lower 16 bits of the first integer
                            x = _xy >> 16  # Extract the upper 16 bits for x
                            y = _xy & 0xFFFF  # Extract the lower 16 bits for y
                            # Convert units from integer to appropriate scale
                            x = float(x) / 10.0  # 0.1 deg to degree
                            y = float(y) / 10.0  # mm to cm
                            gyro = float(_gyro)/10.0  # gyro in 0.1 deg/s to deg/s
                            linaccel = float(_linaccel) / 100.0  # cm/s^2 to m/s^2
                            diag_input = _i / 10.0  # PID P value
                            diag_output = _o / 10.0  # PID I value
                            diag_output_sum = _os / 10.0  # PID D value
                            diag_e = _e / 10.0  # PID error value
                            # logger.info("Tracker Point %d: x=%f, y=%f, status=%d, linaccel=%f, echo=%d, gyro=%f",
                            #                 i, x, y, status, linaccel, echo, gyro)
                            tracker_data.append((x, y, status, linaccel, echo, gyro,
                                                 diag_input, diag_output, diag_output_sum, diag_e))
                        except struct.error as e:
                            logger.error("Error unpacking tracker data at index %d: %s", i, e)
                            continue

                    # Log the tracker data
                    # if n_tracker_points > 0:
                    #     for track_point in tracker_data:
                    #         logger.info("%s: %r", "Tracker Data", track_point)
                            
                    # Create a dictionary to hold the telemetry data
                    telemetry_data = {
                        'telemetry_avail_flag': telemetry_avail_flag,
                        'n_tracker_points': n_tracker_points,
                        'sys_time': sys_time,
                        'left_speed': left_speed,
                        'right_speed': right_speed,
                        'heading': heading,
                        'tracker_data': tracker_data
                    }
                    
                    data_queue.put(['TELE_AND_TRACK', telemetry_data])
                    

                    
                elif (characteristic.uuid == "8cf10e3b-0e9c-4809-b94a-5217ed9d6902".lower()):
                    # logger.info("%s: %r", "Message from car: ", data)
                    message = data.split(b'\0')[0].decode()
                    if (message.startswith("DATA:")):
                        data_str = message.split(':')[1]
                        # data_str = data_str.split(',')
                        # data_arr = np.array(data_str, dtype=np.float32)
                        data_queue.put(['DATA', data_str])
                    elif (message.startswith("RANGE:")):
                        data_str = message.split(':')[1]
                        data_str = data_str.split(',')
                        data_arr = np.array(data_str, dtype=np.float32)
                        data_queue.put(['RANGE', data_arr])
                    elif (message.startswith("TELE:")):
                        # This is a telemetry message, we will parse it as JSON
                        value = message[5:]  # Skip the "TELE:" prefix
                        glob_model['telemetry_str_len'] = len(value)
                        # logger.info("%s: %r", "Expaned Telemetry", value)
                        try:
                            json_data = json.loads(value)
                        except JSONDecodeError:
                            logger.warning("%s: %r", "JSON Decode error", value)
                            return
                        telemetry_Queue.put(json_data)
                        glob_model['last_telemetry_time'] = time.monotonic()

            glob_BLE_connected = 2

            await client.start_notify("19B10001-E8F2-537E-4F6C-D104768A1214", notification_handler)
            await client.start_notify("8cf10e3b-0e9c-4809-b94a-5217ed9d6902", notification_handler)

            glob_model['last_telemetry_time'] = time.monotonic()

            while True:
                global glob_UI_disconnected
                
                if glob_UI_disconnected:
                    print("UI disconnect detected, ending comm thread")
                    break
                
                if ((time.monotonic() - glob_model['last_telemetry_time']) > 2):
                    print(time.monotonic, glob_model['last_telemetry_time'])
                    print("Telemetry lost, disconnecting")
                    raise DeviceNotFoundError("Connection lost with: {}".format(target_name))

                if (not input_queue.empty()):
                    message = input_queue.get()
                    message += "\0"
                    message = message.encode()
 
                    await client.write_gatt_char('99924646-b9d6-4a51-bda9-ef084d793abf', message)

                await asyncio.sleep(0.01)
                
            

    else:
        glob_BLE_connected = 0
        raise DeviceNotFoundError("No device found with name: {}".format(target_name))



def backend_init():

        glob_model['is_init'] = True
        glob_model['CAR_NAME'] = DEFAULT_CAR_NAME
        glob_model['data'] = {}
        glob_model['BLE_Task'] = None
        glob_model['ble_rssi'] = 0
        glob_model['_prev_txt_cmd_message'] = ''
        glob_model['car_state'] = 0
        glob_model['car_heading'] = 0
        glob_model['car_heading_tracking_latch'] = 0

        
def backend_connect():
    glob_model['BLE_Task'] = asyncio.create_task(ble_task(iq, oq, tq, dq))
        
def backend_disconnect():
    global glob_UI_disconnected
    global glob_BLE_connected
    glob_UI_disconnected = 1
    glob_BLE_connected = 1
        
def backend_send_msg():
    message = comm_text_input.value
    glob_model['_prev_txt_cmd_message'] = message
    comm_text_input.value = ''
    backend_enqueue_message(message)

def backend_comm_text_prev_message():
    comm_text_input.value = glob_model['_prev_txt_cmd_message']
        
def backend_enqueue_message(message, queue_on_empty=False):
    if glob_BLE_connected:
        if queue_on_empty:
            if (not iq.empty()):
                return
        iq.put(message)
    
def backend_set_car_name(e):
    glob_model['CAR_NAME'] = e.value

def backend_car_direction_mode_sw_change():
    if car_direction_mode_sw.value:
        backend_enqueue_message('car_set_mode 2')
    else:
        backend_enqueue_message('car_set_mode 0')
        car_direction_label.text = 'Car not in heading mode'

def backend_car_direction_joystick_update(e):
    if car_direction_mode_sw.value:
        angle = 360 - (math.atan2(e.x,e.y * -1)/math.pi*180 + 180)
        glob_model['heading_joystic_angle'] = round(angle, 2)
        car_direction_label.text = 'heading: {} degrees'.format( glob_model['heading_joystic_angle'])

def backend_car_direction_joystick_set():
    if car_direction_mode_sw.value:
        angle = glob_model['heading_joystic_angle']
        backend_enqueue_message('car_set_heading {}'.format(angle))
        

def backend_move_quick_reverse(e):
    cmd_str = 'car_m_move {} {} {}'.format(-200, -200, 750)
    backend_enqueue_message(cmd_str)
    
def backend_manual_move_click():
    left_speed = manual_control_left_slider.value
    right_speed = manual_control_right_slider.value
    duration = manual_control_duration_slider.value

    cmd_str = 'car_m_move {} {} {}'.format(left_speed, right_speed, duration)
    print(cmd_str)
    backend_enqueue_message(cmd_str)
    
def backend_auto_mode_click():
    heading = auto_control_heading_slider.value
    speed = int(auto_control_speed_slider.value)
    duration = auto_control_duration_slider.value
    
    duration = int(duration * 1000)
    
    cmd_str = 'car_m_auto {} {} {}'.format(heading, speed, duration)
    backend_enqueue_message(cmd_str)

def backend_idle_mode_click():
    # This is a special command to put the car in idle mode, which is not the same as stop
    # it will stop the motors, but will not reset the heading
    backend_enqueue_message('car_set_mode 0')
    
def backend_slew_servo(e):
    angle = servo_angle_slider.value
    backend_enqueue_message('car_set_servo {}'.format(angle))
    
def backend_ranging_click():
    backend_enqueue_message('car_do_ranging')
    
def backend_ping_click():
    backend_enqueue_message('car_ping')
    
def backend_clear_tracker_click():
    tracker_chart.options['series'][0]['data'] = [[0, 0]]
    glob_model['car_heading_tracking_latch'] = glob_model['car_heading']
    tracker_chart.update()
    range_chart.options['series'][0]['data'] = [[0, None]]
    range_chart.options['series'][1]['data'] = [[0, None]]
    state_chart.options['series'][0]['data'] = [[0, None]]
    gyro_chart.options['series'][0]['data'] = [[0, None]]
    range_chart.update()
    state_chart.update()
    gyro_chart.update()
    
def backend_update():
    
    global glob_BLE_connected

    if glob_model['is_init']:

        if (glob_model['BLE_Task'] is not None):
            if (glob_model['BLE_Task'].done()):
                if (glob_model['BLE_Task'].exception()):
                    print("BLE Task exception")
                    ex = glob_model['BLE_Task'].exception()
                    template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                    message = template.format(type(ex).__name__, ex.args)
                    print(message)
                    ui.notify(message)
                    glob_model['BLE_Task'] = None
                glob_BLE_connected = 0

        
        if (not oq.empty()):
            # log_str = '{:12d}:\t{}'.format(glob_model['data']['time_ms'],
            #                              glob_model['data']['message'])
            log_str = oq.get()
            comm_log.push(log_str)
            
        if (not dq.empty()):
            data_packet = dq.get()
            if data_packet[0] == 'DATA':
                data_log.push(data_packet[1])
            elif data_packet[0] == 'RANGE':
                ranging_data = data_packet[1]
                temp_list_0 = []
                temp_list_1 = []
                for range in ranging_data:
                    if range < 0:
                        temp_list_0.append('-')
                        temp_list_1.append(1)
                    else:
                        temp_list_0.append(range)
                        temp_list_1.append('-')
                ranging_chart.options['series'][0]['data'] = temp_list_0
                ranging_chart.options['series'][1]['data'] = temp_list_1
                ranging_chart.update()
            elif data_packet[0] == 'TELE_AND_TRACK':
                telemetry_data = data_packet[1]
                telemetry_table_rows[0]['value'] = telemetry_data['sys_time']
                telemetry_table_rows[1]['value'] = telemetry_data['left_speed']
                telemetry_table_rows[2]['value'] = telemetry_data['right_speed']
                telemetry_table_rows[3]['value'] = telemetry_data['heading']
                glob_model['car_heading'] = telemetry_data['heading']
                telemetry_table.update()
                
                for tracking_point in telemetry_data['tracker_data']:
                    
                    #location is relateive to the car, so necessary to add to the last point
                    last_xy = tracker_chart.options['series'][0]['data'][-1]
                    
                    angle_rad = math.radians(tracking_point[0] - glob_model['car_heading_tracking_latch'])  # Convert angle to radians, relative to the car heading
                    # polar coordinate has the 0 degree pointing to the right and advance counter-clockwise
                    # we wish to represent the 0 degree direction as the y axis, and 90 degrees as the postive x-axis
                    # so we need to rotate the angle by 90 degrees
                    angle_rad = angle_rad + math.pi/2
                    # Calculate the x and y offsets using polar coordinates
                    
                    # distance has already been converted to cm in the telemetry packet
                    dx = tracking_point[1] * math.cos(angle_rad) * -1.0 # Calculate x-offset
                    dy = tracking_point[1] * math.sin(angle_rad)  # Calculate y-offset
                    # Calculate the new coordinates and rotate so the 0 degrees is at the top
                    # of the chart
                    
                    
                    x = dx + last_xy[0]  # Calculate x-coordinate
                    y = dy + last_xy[1] # Calculate y-coordinate
                    track_point_xy = [x, y]
                    tracker_chart.options['series'][0]['data'].append(track_point_xy)
                    
                    sample_number = max(range_chart.options['series'][0]['data'][-1][0],
                                        range_chart.options['series'][1]['data'][-1][0])

                    range_point = [sample_number + 1, tracking_point[4]]
                    range_point_accel = [sample_number+ 1, tracking_point[3]]
                    
                    gyro_point = [sample_number + 1, tracking_point[5]]
                    gyro_chart.options['series'][0]['data'].append(gyro_point)

                    # Only add echo point if it is valid
                    status_byte = tracking_point[2]


                    # decode bits 3 to 7 as a uint8
                    auto_mode_state = (status_byte & 0xF8) >> 3

                    # if bit 2 is set, the car is in auto mode
                    if (status_byte & 0x04):
                        # car is in auto mode
                        state_point = [sample_number + 1, auto_mode_state]
                    else:
                        state_point = [sample_number + 1, None]
                    state_chart.options['series'][0]['data'].append(state_point)

                    # echo is valid if bit 1 is set
                    if (status_byte & 0x02):
                        # echo is valid, add to the chart
                        range_chart.options['series'][0]['data'].append(range_point)
                    else:
                        # echo is not valid, add a placeholder
                        range_chart.options['series'][0]['data'].append([sample_number + 1, None])
                    range_chart.options['series'][1]['data'].append(range_point_accel)
                    
                    # add PID diagnostic values to the log
                    pid_diagnostic_log_str = 'PID Diagnostic: Sample: {:<5} |Input: {:>7.1f} |Output: {:>7.1f} |Output Sum: {:>7.1f} |Error: {:>7.1f}'.format(
                        sample_number, tracking_point[6], tracking_point[7], tracking_point[8], tracking_point[9])
                    pid_diagnostic_log.push(pid_diagnostic_log_str)

                # only update the chart if there are new data
                # prevents the chart from resetting the zoom settings
                if len(telemetry_data['tracker_data']) > 0:
                    tracker_chart.update()
                    range_chart.update()
                    state_chart.update()
                    gyro_chart.update()

        if (not tq.empty()):
            glob_model['data'] = tq.get()
            
            glob_model['ble_rssi'] = float(glob_model['data']['ble_rssi']) * -1.0
            if (glob_model['ble_rssi'] == -127):
                glob_model['ble_rssi'] = 100
                
            glob_model['cycles_in_telemetry'] = float(glob_model['data']['n_cycles'])
            glob_model['mcu_times_since_telemetry'] = float(glob_model['data']['t_last'])

            glob_model['car_state'] = int(glob_model['data']['CAR']['mode'])

def backend_medium_update():
    if glob_model['is_init']:

        pass


def backend_slow_update():
    if glob_model['is_init']:
        
        json_view.properties['content'] = {'json' : glob_model['data']}
        json_view.update()
            
                
        if (glob_model['ble_rssi'] != 100):
            
            rssi_value = round((100 - glob_model['ble_rssi'])/60, 2)
            rssi_bar.value = rssi_value
        
        telemetry_lengh_bar.value = round(glob_model['telemetry_str_len']/TELEMETRY_LENGTH, 2)
        
        if glob_model['cycles_in_telemetry'] != 0:
            
            cycle_time = glob_model['mcu_times_since_telemetry']/glob_model['cycles_in_telemetry']
            cycle_time = round(cycle_time/100, 2)
        else:
            cycle_time = 0

        cycle_time_bar.value = cycle_time
        

        if glob_BLE_connected == 0:
            is_disconnected_chip.set_visibility(True)
            is_connecting_chip.set_visibility(False)
            is_connected_chip.set_visibility(False)
        elif glob_BLE_connected == 1:
            is_disconnected_chip.set_visibility(False)
            is_connecting_chip.set_visibility(True)
            is_connected_chip.set_visibility(False)
        elif glob_BLE_connected == 2:
            is_disconnected_chip.set_visibility(False)
            is_connecting_chip.set_visibility(False)
            is_connected_chip.set_visibility(True)
        else:
            is_disconnected_chip.set_visibility(False)
            is_connecting_chip.set_visibility(False)
            is_connected_chip.set_visibility(False)

def create_comm_window():
    global comm_log, comm_text_input
    with ui.card() as comm_window:
        comm_window.tight()
        comm_window.classes('w-full bg-gray-100')
        with ui.card_section():
            ui.label('Communication Window')
        comm_log = ui.log(max_lines=100)
        comm_log.classes('w-full')
        comm_log.style('font-size: 75%; white-space: pre-wrap;')
        comm_text_input = ui.input(label='Enter commands here:')
        comm_text_input.classes('w-full h-20 bg-gray-300')
        comm_text_input.on('keydown.enter', backend_send_msg)
        comm_text_input.on('keydown.up', backend_comm_text_prev_message)
    return comm_window


def create_tracker_window():
    global tracker_chart
    with ui.card() as tracker_window:
        tracker_window.tight()
        tracker_window.classes('w-full')

        tracker_chart = ui.echart({
            'title': {'text': 'Tracker Data'},
            'tooltip': {'trigger': 'item', 'axisPointer': {'type': 'cross'}},
            'dataZoom': [
                { 
                    'type': 'inside',
                },
                {
                    'type': 'inside',
                    'yAxisIndex': 0,
                }
            ],
            'xAxis': {
                'type': 'value',
                'name': 'X (cm)',
                'nameLocation': 'middle',
                'min': -200,
                'max': 200,
                'scale': True,
                'axisLabel': {'formatter': '{value}'},
            },
            'yAxis': {
                'type': 'value',
                'name': 'Y (cm)',
                'nameLocation': 'middle',
                'min': -200,
                'max': 200,
                'scale': True,
                'axisLabel': {'formatter': '{value}'},
            },
            'series': [{'type': 'scatter', 'data': [[0, 0]], 'symbolSize': 5}],
        })
        tracker_chart.classes('w-full h-full')
        
        


        with ui.card_section():
            ui.button('Clear plot / Re-Orient', on_click=backend_clear_tracker_click)
    return tracker_window

def create_data_window():
    global range_chart, state_chart
    with ui.card() as tracker_window:
        tracker_window.tight()
        tracker_window.classes('w-full')
        range_chart = ui.echart({
            'title': {'text': 'Ranging and Acceleration Data'},
            'tooltip': {'trigger': 'item', 'axisPointer': {'type': 'cross'}},
            'dataZoom': [
                {
                    'type': 'inside',
                    'start': 0,
                    }
                ],
            'xAxis': {
                'type': 'value',
                'name': 'Sample Number',
                'nameLocation': 'middle',
                'scale': True,
                'axisLabel': {'formatter': '{value}'},
            },
            'yAxis': [
                {
                    'type': 'value',
                    'name': 'Y (cm)',
                    'nameLocation': 'middle',
                    'min': 0,
                    'max': 100,
                    'scale': True,
                    'axisLabel': {'formatter': '{value}'},
                },
                {
                    'type': 'value',
                    'name': 'Acceleration',
                    'nameLocation': 'middle',
                    'color': 'purple',
                    'min': -3,
                    'max': 3,
                    'scale': True,
                    'alignTicks': True,
                    'axisLabel': {'formatter': '{value}'},
                    'position': 'right',
                },
            ],
            'series': [
                {'type': 'line', 'name': 'Ranging', 'data': [[0, None]], 'yAxisIndex': 0, 'symbolSize': 5},
                {'type': 'line', 'name': 'Acceleration', 'data': [[0, None]], 'symbolSize': 2, 'yAxisIndex': 1, 'color': 'red'},
            ],
        })

        state_chart = ui.echart({
            'title': {'text': 'Auto Mode State Data'},
            'tooltip': {'trigger': 'item', 'axisPointer': {'type': 'cross'}},
            'dataZoom': [
                {
                    'type': 'inside',
                    'start': 0,
                    }
                ],
            'xAxis': {
                'type': 'value',
                'name': 'Sample Number',
                'nameLocation': 'middle',
                'scale': True,
                'axisLabel': {'formatter': '{value}'},
            },
            'yAxis': [
                {
                    'type': 'value',
                    'name': 'State Number',
                    'nameLocation': 'middle',
                    'scale': True,
                    'axisLabel': {'formatter': '{value}'},
                },
            ],
            'series': [
                {'type': 'line', 'name': 'State', 'data': [[0, None]], 'yAxisIndex': 0, 'symbolSize': 5},
            ],
        })
        
        
def create_gyro_window():
    global gyro_chart
    with ui.card() as gyro_window:
        gyro_window.tight()
        gyro_window.classes('w-full h-80')

        gyro_chart = ui.echart({
            'title': {'text': 'Gyro Data'},
            'tooltip': {'trigger': 'item', 'axisPointer': {'type': 'cross'}},
            'dataZoom': [
                {
                    'type': 'inside',
                    'start': 0,
                    }
                ],
            'xAxis': {
                'type': 'value',
                'name': 'Sample Number',
                'nameLocation': 'middle',
                'scale': True,
                'axisLabel': {'formatter': '{value}'},
            },
            'yAxis': [
                {
                    'type': 'value',
                    'name': 'Gyro rate (deg/s)',
                    'nameLocation': 'middle',
                    'scale': True,
                    'axisLabel': {'formatter': '{value}'},
                },
            ],
            'series': [
                {'type': 'line', 'name': 'gyro', 'data': [[0, None]], 'yAxisIndex': 0, 'symbolSize': 5},
            ],
        })
        gyro_chart.classes('w-full h-full')

    return gyro_window

def create_auto_mode_card():
    global auto_control_heading_slider, auto_control_speed_slider, auto_control_duration_slider
    with ui.card() as auto_card:
        auto_card.tight()
        auto_card.classes('w-11/12 bg-green-200')
        with ui.card_section():
            ui.markdown('###Auto Mode Settings')
        with ui.grid(columns='1fr 2fr').classes('w-11/12'):
            ui.label('Heading')
            auto_control_heading_slider = ui.slider(min=-180, max=180, step=1, value=0).props('label-always')
            ui.label('Forward Speed')
            auto_control_speed_slider = ui.slider(min=0, max=255, step=1, value=200).props('label-always')
            ui.label('Duration (s)')
            auto_control_duration_slider = ui.slider(min=0, max=10, step=0.2, value=2).props('label-always')
        with ui.card_section():
            ui.button('Execute!', on_click=backend_auto_mode_click)
    return auto_card


def create_front_sensor_card():
    global servo_angle_slider
    with ui.card() as front_sensor_card:
        front_sensor_card.tight()
        front_sensor_card.classes('w-11/12 h-80 bg-blue-200')
        with ui.card_section():
            ui.markdown('###Front Sensor')
        with ui.grid(columns='1fr 2fr').classes('w-11/12'):
            ui.label('Sensor Angle')
            servo_angle_slider = ui.slider(min=-90, max=90, step=1, value=0).props('label-always') \
                .on('change', backend_slew_servo)
        with ui.card_section():
            ui.button('One Ping Only', on_click=backend_ping_click)
    return front_sensor_card


def create_manual_move_card():
    global manual_control_left_slider, manual_control_right_slider, manual_control_duration_slider
    with ui.card() as manual_move_card:
        manual_move_card.tight()
        manual_move_card.classes('w-11/12 h-80 bg-yellow-200')
        with ui.card_section():
            ui.markdown('###Manual control')
        with ui.grid(columns='1fr 2fr').classes('w-11/12'):
            ui.label('Left Speed')
            manual_control_left_slider = ui.slider(min=-255, max=255, step=1, value=200).props('label-always')
            ui.label('Right Speed')
            manual_control_right_slider = ui.slider(min=-255, max=255, step=1, value=200).props('label-always')
            ui.label('Duration (ms)')
            manual_control_duration_slider = ui.slider(min=0, max=5000, step=1, value=1000).props('label-always')
        with ui.card_section():
            ui.button('Execute!', on_click=backend_manual_move_click)
    return manual_move_card


def create_ranging_card():
    global ranging_chart
    with ui.card() as ranging_card:
        ranging_card.tight()
        ranging_card.classes('w-11/12 h-80 bg-blue-200')
        ranging_chart = ui.echart({
            'title': {'text': 'Ranging Data'},
            'polar': {'radius': [5, '60%']},
            'angleAxis': {
                'type': 'category',
                'data': ['-90°', '-60°', '-30°', '0°', '30°', '60°', '90°'],
                'startAngle': 195,
                'endAngle': -15
            },
            'radiusAxis': {'min': 0, 'max': 1},
            'series': [
                {'type': 'bar', 'data': [1, 1, 1, 1, 1, 1, 1], 'coordinateSystem': 'polar', 'stack': 'a'},
                {'type': 'bar', 'data': ['-', '-', '-', '-', '-', '-', '-'], 'coordinateSystem': 'polar', 'stack': 'a', 'color': 'red'}
            ]
        })
        with ui.card_section():
            ui.button('Scan', on_click=backend_ranging_click)
    return ranging_card


def create_heading_card():
    global car_direction_mode_sw, car_direction_label
    with ui.card() as heading_card:
        heading_card.tight()
        heading_card.classes('w-11/12 bg-green-200')
        with ui.card_section():
            car_direction_mode_sw = ui.switch('Heading Control Mode', on_change=backend_car_direction_mode_sw_change)
            joystick_direction = ui.joystick(color='green', size=50,
                                            on_move=backend_car_direction_joystick_update,
                                            on_end=backend_car_direction_joystick_set)
            joystick_direction.classes('w-full h-40')
        with ui.card_section():
            car_direction_label = ui.label('Car not in heading mode')
    return heading_card

def create_pid_card():
    global pid_kp_slider, pid_ki_slider, pid_kd_slider
    with ui.card() as pid_card:
        pid_card.tight()
        pid_card.classes('w-11/12 bg-yellow-200')
        with ui.card_section():
            ui.markdown('###PID Control')
        with ui.grid(columns='1fr 2fr').classes('w-11/12'):
            ui.label('Kp')
            pid_kp_slider = ui.slider(min=0, max=10, step=0.1, value=1).props('label-always')
            ui.label('Ki')
            pid_ki_slider = ui.slider(min=0, max=10, step=0.1, value=0).props('label-always')
            ui.label('Kd')
            pid_kd_slider = ui.slider(min=0, max=10, step=0.1, value=0).props('label-always')
        with ui.card_section():
            ui.button('Set PID', on_click=lambda: backend_enqueue_message(
                'car_set_pid {} {} {}'.format(pid_kp_slider.value, pid_ki_slider.value, pid_kd_slider.value)))
            
    return pid_card

def create_pid_diagnostic_card():
    global pid_diagnostic_log
    with ui.card() as pid_diagnostic_card:
        pid_diagnostic_card.tight()
        pid_diagnostic_card.classes('w-11/12 bg-gray-100')
        with ui.card_section():
            ui.markdown('###PID Diagnostic Log')
        pid_diagnostic_log = ui.log(max_lines=100)
        pid_diagnostic_log.classes('w-full')
        pid_diagnostic_log.style('font-size: 55%; white-space: pre-wrap;')
    return pid_diagnostic_card

log_level = logging.INFO
logging.basicConfig(
    level=log_level,
    format="%(levelname)s: %(message)s",
)

# UI CODE STARTS HERE ======================

app.native.start_args['debug'] = False
app.native.settings['ALLOW_DOWNLOADS'] = True
app.on_disconnect(backend_disconnect)

ui.timer(0, callback=backend_init, once=True)
ui.timer(0.1, callback=backend_update)
ui.timer(0.3, callback=backend_medium_update)
ui.timer(0.5, callback=backend_slow_update)


with ui.header(elevated=True).style('background-color: #3874c8').classes('items-center justify-between'):

    with ui.row().classes('w-full'):
        
        ui.icon('directions_car').classes('text-white text-3xl')
        ui.label('Robot Car Control').classes('text-white text-2xl font-bold')

        ui.label(VERSION_STR)
        ui.button('ESTOP', icon='block', color='red', on_click=backend_idle_mode_click).props('flat')

        ui.space()
        
        ip_textbox = ui.input(label='Enter the name of the car to connect',
                                value=DEFAULT_CAR_NAME,
                                on_change=backend_set_car_name)
        
        ip_textbox.on('keydown.enter', backend_connect) 
        

        is_disconnected_chip = ui.chip("Offline",
                                    icon='block',
                                    color='red')
        is_disconnected_chip.set_enabled(False)
        is_connected_chip = ui.chip('Connected',
                                    icon='thumb_up',
                                    color='green')
        is_connected_chip.set_enabled(False)
        is_connected_chip.set_visibility(False)
        is_connecting_chip = ui.chip('Connecting',
                                    icon='history',
                                    color='orange')
        is_connecting_chip.set_enabled(False)
        is_connecting_chip.set_visibility(False)
        

with ui.right_drawer(top_corner=True, bottom_corner=True) as right_hand_drawer:
    right_hand_drawer.style('background-color: #d7e3f4')
    right_hand_drawer.props('width=350')
    ui.markdown('##Robot Status')
    
    with ui.tabs().classes('w-full') as tabs:
        control = ui.tab('Control')
        vis = ui.tab('Data')
        raw = ui.tab('Telemetry')
        cli = ui.tab('CLI')
        
        
    with ui.tab_panels(tabs, value=control).classes('w-full'):
        with ui.tab_panel(vis):

            with ui.grid(columns='1fr 1fr').classes('h-6/12 gap-0'):
                
                ui.label("Signal Strength")
                rssi_bar = ui.linear_progress()
                
                ui.label("Telemetry Buffer")
                telemetry_lengh_bar = ui.linear_progress()
                
                ui.label("Cycle time")
                cycle_time_bar = ui.linear_progress()
            
            ui.separator()
            telemetry_table_columns = [
                    {'name': 'Field', 'label': 'Field', 'field': 'field', 'required': True, 'align': 'left'},
                    {'name': 'Value', 'label': 'Value', 'field': 'value', 'align': 'right'},
                ]
            
            telemetry_table_rows = [
                    {'field': 'System Time', 'value': 0},
                    {'field': 'Left Speed', 'value': 0},
                    {'field': 'Right Speed', 'value': 0},
                    {'field': 'Heading', 'value': 0},
                ]
            
            telemetry_table = ui.table(columns=telemetry_table_columns, rows=telemetry_table_rows, row_key='field')
            telemetry_table.classes('w-full')
            telemetry_table.style('font-size: 75%;')
                   
            ui.separator()
            ui.button('Clear plot / Re-Orient', on_click=backend_clear_tracker_click)
            ui.separator()
            
            with ui.card() as data_window:
                data_window.tight()
                data_window.classes('w-full')
                with ui.card_section():
                    ui.markdown('###Data Window')
                data_log = ui.log(max_lines=200)
                data_log.classes('w-full h-4/12')
                data_log.style('font-size: 55%;')
                
        with ui.tab_panel(raw):
            ui.separator()
            ui.markdown('###Telemetry Data')
            json_view = ui.json_editor({'content': {'json': {}}})
            json_view.classes(('w-fit h-fit'))

        with ui.tab_panel(cli):
            ui.separator()
            ui.markdown('###Command Line Interface')
            comm_window = create_comm_window()
            
        with ui.tab_panel(control):
            ui.separator()
            ui.markdown('###Control Panel')
            auto_card = create_auto_mode_card()
            front_sensor_card = create_front_sensor_card()
            manual_move_card = create_manual_move_card()
            heading_card = create_heading_card()
            ranging_card = create_ranging_card()
            
            
            # with ui.grid(columns='1fr 1fr').classes('w-full gap-0'):
            #     auto_card = create_auto_mode_card()
            #     front_sensor_card = create_front_sensor_card()
                
            # with ui.grid(columns='1fr 1fr').classes('w-full gap-0'):
            #     manual_move_card = create_manual_move_card()
            #     heading_card = create_heading_card()
                
            # with ui.card() as ranging_card:
            #     ranging_card.tight()
            #     ranging_card.classes('w-full h-80 bg-blue-200')
            #     ranging_chart = create_ranging_card()



with ui.grid(columns='2fr 2fr').classes('w-full gap-0'):
    tracker_window = create_tracker_window()
    data_window = create_data_window()

ui.separator()  
gyro_window = create_gyro_window()
ui.separator()  
pid_diag = create_pid_diagnostic_card()

# with ui.grid(columns='2fr 1fr').classes('w-full gap-0'):



glob_model['is_ui_init'] = True



ui.run(native=True, window_size=(1100, 900), fullscreen=False, reload=False, port=native.find_open_port())

# To build as exe, run the following in command line (need pyinstaller)
#  cd .\Codes\Python\Remte_Control_Client\
#  nicegui-pack --onefile --windowed --name "RCC" main.py