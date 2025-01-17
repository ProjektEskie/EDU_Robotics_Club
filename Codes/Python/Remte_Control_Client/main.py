# Requires the following python libraries:
# numpy nicegui pywebview bleak

import threading
import queue
import json
from nicegui import app, ui, native
import asyncio
import math
import numpy as np
from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
import logging
import time

logger = logging.getLogger(__name__)

iq = queue.SimpleQueue()
oq = queue.SimpleQueue()
tq = queue.SimpleQueue()
dq = queue.SimpleQueue()

DEFAULT_CAR_NAME = 'RClub_Car'

VERSION_STR = '2.7'

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

                    value = await client.read_gatt_char("7b0db1df-67ed-46ef-b091-b4472119ef6d")
                    value = value.decode().split('\0')[0]
                    glob_model['telemetry_str_len'] = len(value)
                    # logger.info("%s: %r", "Expaned Telemetry", value)
                    json_data = json.loads(value)
                    telemetry_Queue.put(json_data)
                    glob_model['last_telemetry_time'] = time.monotonic()
                    
                elif (characteristic.uuid == "8cf10e3b-0e9c-4809-b94a-5217ed9d6902".lower()):
                    # logger.info("%s: %r", "Message from car: ", data)
                    message = data.decode().split('\0')[0]
                    if (message.startswith("DATA:")):
                        data_str = message.split(':')[1]
                        # data_str = data_str.split(',')
                        # data_arr = np.array(data_str, dtype=np.float32)
                        data_queue.put(data_str)
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
    
def backend_slew_servo(e):
    angle = servo_angle_slider.value
    backend_enqueue_message('car_set_servo {}'.format(angle))
    
    
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
            data_str = dq.get()
            data_log.push(data_str)

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
        
        telemetry_lengh_bar.value = round(glob_model['telemetry_str_len']/600, 2)
        
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
    right_hand_drawer.props('width=500')
    ui.markdown('##Robot Status')
    
    with ui.tabs().classes('w-full') as tabs:
        vis = ui.tab('Data')
        raw = ui.tab('Telemetry')
        
    with ui.tab_panels(tabs, value=raw).classes('w-full'):
        with ui.tab_panel(vis):

            ui.markdown('###Bars')
            with ui.grid(columns='1fr 1fr').classes('h-6/12 gap-0'):
                
                ui.label("Signal Strength")
                rssi_bar = ui.linear_progress()
                
                ui.label("Telemetry Buffer")
                telemetry_lengh_bar = ui.linear_progress()
                
                ui.label("Cycle time")
                cycle_time_bar = ui.linear_progress()
                
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

    with ui.card() as front_sensor_card:
        front_sensor_card.tight()
        front_sensor_card.classes('w-11/12 h-80 bg-blue-200')
        
        with ui.card_section():
            ui.markdown('###Front Sensor')
            
        with ui.grid(columns='1fr 2fr').classes('w-11/12'):
            ui.label('Sensor Angle')
            servo_angle_slider = ui.slider(min=-90, max=90, step=1, value=0).props('label-always') \
            .on('change', backend_slew_servo)

glob_model['is_ui_init'] = True



ui.run(native=True, window_size=(1100, 900), fullscreen=False, reload=False, port=native.find_open_port())

# To build as exe, run the following in command line (need pyinstaller)
#  nicegui-pack --onefile --windowed --name "RCC" main.py