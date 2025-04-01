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
TELEMETRY_LENGTH = 480
VERSION_STR = '2.13'

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

                    # The struct format string corresponds to the telemetry packet structure
                    # Format: telemetry_avail_flag (1 byte), n_tracker_points (1 byte),
                    # spare_bytes (10 bytes), sys_time (4 bytes), left_speed (4 bytes),
                    # right_speed (4 bytes), heading (4 bytes)
                    struct_format = "<BB10xIiiI"

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
                    tracker_data_format = "<llB3x"  # Each tracker point consists of two integers, one byte and 3 padding bytes (x, y, r)
                    tracker_data_size = struct.calcsize(tracker_data_format)
                    fixed_part_size = struct.calcsize(struct_format)  # Size of the fixed part of the struct

                    for i in range(n_tracker_points):
                        offset = fixed_part_size + i * tracker_data_size  # Fixed part is 20 bytes, then tracker points follow
                        try:
                            _xy, linaccel, r = struct.unpack_from(tracker_data_format, data, offset)
                            # x position is stored as the upper 16 bits of the first integer, y position is stored as the lower 16 bits of the first integer
                            x = _xy >> 16  # Extract the upper 16 bits for x
                            y = _xy & 0xFFFF  # Extract the lower 16 bits for y
                            # Convert units from integer to appropriate scale
                            x = float(x) / 10.0  # 0.1 deg to degree
                            y = float(y) / 10.0  # mm to cm
                            linaccel = float(linaccel) / 100.0  # cm/s^2 to m/s^2
                            tracker_data.append((x, y, r, linaccel))
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
                    
                    # Then we perform a read for the extended telemetry packet, the car
                    # will pause transmission of the messaging packet for 2 cycles (~100ms)
                    # to give airtime for this extended packet to be read.
                    value = await client.read_gatt_char("7b0db1df-67ed-46ef-b091-b4472119ef6d")
                    value = value.split(b'\0')[0].decode()
                    glob_model['telemetry_str_len'] = len(value)
                    # logger.info("%s: %r", "Expaned Telemetry", value)
                    try:
                        json_data = json.loads(value)
                    except JSONDecodeError:
                        logger.warning("%s: %r", "JSON Decode error", value)
                        return
                    telemetry_Queue.put(json_data)
                    glob_model['last_telemetry_time'] = time.monotonic()
                    
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
                    else:
                        output_queue.put(message)

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
    range_chart.options['series'][0]['data'] = [[0, 0]]
    range_chart.options['series'][1]['data'] = [[0, 0]]
    range_chart.update()
    
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
                # telemetry_table_rows = [
                #     {'field': 'System Time', 'value': 0},
                #     {'field': 'Left Speed', 'value': 0},
                #     {'field': 'Right Speed', 'value': 0},
                #     {'field': 'Heading', 'value': 0},
                # ]
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
                    
                    last_range = range_chart.options['series'][0]['data'][-1]
                    range_point = [last_range[0] + 1, tracking_point[2]]
                    last_accel = range_chart.options['series'][1]['data'][-1]
                    range_point_accel = [last_accel[0] + 1, tracking_point[3]]
                    range_chart.options['series'][0]['data'].append(range_point)
                    range_chart.options['series'][1]['data'].append(range_point_accel)
                    
                tracker_chart.update()
                range_chart.update()

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
        
        ui.markdown('#Robot Car Control')
        ui.label(VERSION_STR)

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
        vis = ui.tab('Data')
        raw = ui.tab('Telemetry')
        
    with ui.tab_panels(tabs, value=vis).classes('w-full'):
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
            
            with ui.card() as tracker_window:
                tracker_window.tight()
                tracker_window.classes('w-full')

                    
                tracker_chart = ui.echart({
                    'title': {
                        'text': 'Tracker Data'
                    },
                    'xAxis': {
                        'type': 'value',
                        'name': 'X (cm)',
                        'nameLocation': 'middle',
                        'scale': True,
                        'axisLabel': {
                            'formatter': '{value}'
                        },
                    },
                    'yAxis': {
                        'type': 'value',
                        'name': 'Y (cm)',
                        'nameLocation': 'middle',
                        'scale': True,
                        'axisLabel': {
                            'formatter': '{value}'
                        },
                    },
                    'series': [{
                        'type': 'scatter',
                        'data': [[0, 0]],
                        'symbolSize': 5
                    }]
                })
                
                range_chart = ui.echart({
                    'title': {
                        'text': 'Ranging Data'
                    },
                    'xAxis': {
                        'type': 'value',
                        'name': 'Sample Number',
                        'nameLocation': 'middle',
                        'scale': True,
                        'axisLabel': {
                            'formatter': '{value}'
                        },
                    },
                    'yAxis': [
                        {
                        'type': 'value',
                        'name': 'Y (cm)',
                        'nameLocation': 'middle',
                        'min': 0,
                        'max': 100,
                        'scale': True,
                        'axisLabel': {
                            'formatter': '{value}'
                        },
                    },
                    {
                        'type': 'value',
                        'name': 'Acceleration',
                        'nameLocation': 'middle',
                        'min': -2,
                        'max': 2,
                        'scale': True,
                        'alignTicks': True,
                        'axisLabel': {
                            'formatter': '{value}'
                        },
                        'position': 'right',
                    }
                    ],
                    'series': [{
                        'type': 'line',
                        'data': [[0, 0]],
                        'yAxisIndex': 0,
                        'symbolSize': 5
                    },
                    {
                        'type': 'line',
                        'data': [[0, 0]],
                        'symbolSize': 5,
                        'yAxisIndex': 1,
                        'color': 'red'
                    }]
                })
  
                with ui.card_section():
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



    
with ui.card() as comm_window:
    comm_window.tight()
    comm_window.classes('w-11/12 bg-gray-100')
    with ui.card_section():
        ui.label('Communication Window')
    comm_log = ui.log(max_lines=100)
    comm_log.classes('w-full h-3/12')
    comm_log.style('font-size: 75%;')
    comm_text_input = ui.input(label='Enter commands here:')
    comm_text_input.classes('w-full h-20 bg-gray-300')
    comm_text_input.on('keydown.enter', backend_send_msg)
    comm_text_input.on('keydown.up', backend_comm_text_prev_message)
    
ui.separator()  

with ui.grid(columns='2fr 1fr').classes('w-full gap-0'):
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


            
    with ui.card() as ranging_card:
        ranging_card.tight()
        ranging_card.classes('w-11/12 h-80 bg-blue-200')

        ranging_chart = ui.echart({
            'title': {
                'text': 'Ranging Data'
            },
            'polar': {
                'radius': [5, '60%']
            },
            'angleAxis': {
                'type': 'category',
                'data': ['-90°', '-60°', '-30°', '0°', '30°', '60°', '90°'],
                'startAngle': 195,
                'endAngle': -15
            },
            'radiusAxis': {
                'min': 0,
                'max': 1
            },
            'series': [{
                'type': 'bar',
                'data': [1, 1, 1, 1, 1, 1, 1],
                'coordinateSystem': 'polar',
                'stack': 'a',
            },{
                'type': 'bar',
                'data': ['-', '-', '-', '-', '-', '-', '-'],
                'coordinateSystem': 'polar',
                'stack': 'a',
                'color': 'red'
            }
                       ]
        })
        
        with ui.card_section():
            ui.button('Scan', on_click=backend_ranging_click)
      
    with ui.card() as heading_card:
        heading_card.tight()
        heading_card.classes('w-11/12 bg-green-200')
        with ui.card_section():
            car_direction_mode_sw = ui.switch('Heading Control Mode',
            on_change=backend_car_direction_mode_sw_change)
        joystick_direction = ui.joystick(color='green', size=50,
                                            on_move=backend_car_direction_joystick_update,
                                            on_end=backend_car_direction_joystick_set)
        joystick_direction.classes('w-full h-full')
        with ui.card_section():
            car_direction_label = ui.label('Car not in heading mode')
            


glob_model['is_ui_init'] = True



ui.run(native=True, window_size=(1100, 900), fullscreen=False, reload=False, port=native.find_open_port())

# To build as exe, run the following in command line (need pyinstaller)
#  cd .\Codes\Python\Remte_Control_Client\
#  nicegui-pack --onefile --windowed --name "RCC" main.py