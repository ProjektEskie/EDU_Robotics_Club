import threading
import time
import socket
import queue
import json
from nicegui import app, ui, native
import asyncio
import math
import numpy as np
from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.exc import BleakError
import logging

logger = logging.getLogger(__name__)

iq = queue.SimpleQueue()
oq = queue.SimpleQueue()
tq = queue.SimpleQueue()

DEFAULT_CAR_NAME = 'RClub_Car'
DEFAULT_MAUAL_SPEED='180'

VERSION_STR = '2.1'

TESTING_MODE = False

glob_model = {}
glob_model['is_init'] = False
glob_model['is_ui_init'] = False
glob_model['is_connected_chip'] = None
glob_model['is_disconnected_chip'] = None
glob_model['joystick'] = [0,0,0]
glob_model['joystick_request_speed'] = [0,0]
glob_model['joystick_car_speed'] = DEFAULT_MAUAL_SPEED
glob_model['calibration_plot_data'] = queue.SimpleQueue()
glob_init_semaphore = threading.Semaphore()
glob_UI_disconnected = 0
glob_BLE_connected = 0

class DeviceNotFoundError(Exception):
    pass

async def ble_task(input_queue, output_queue, telemetry_Queue):
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
                    value = value.decode().strip().rstrip('\x00')
                    # logger.info("%s: %r", "Expaned Telemetry", value)
                    json_data = json.loads(value)
                    telemetry_Queue.put(json_data)
                    
                elif (characteristic.uuid == "8cf10e3b-0e9c-4809-b94a-5217ed9d6902".lower()):
                    # logger.info("%s: %r", "Message from car: ", data)
                    output_queue.put(data.decode().split('\0')[0])

            glob_BLE_connected = 2

            await client.start_notify("19B10001-E8F2-537E-4F6C-D104768A1214", notification_handler)
            await client.start_notify("8cf10e3b-0e9c-4809-b94a-5217ed9d6902", notification_handler)

            while True:
                global glob_UI_disconnected
                if glob_UI_disconnected:
                    print("UI disconnect detected, ending comm thread")
                    break

                if (not input_queue.empty()):
                    message = input_queue.get()
                    message += "\0"
                    message = message.encode()
 
                    await client.write_gatt_char('99924646-b9d6-4a51-bda9-ef084d793abf', message)

                await asyncio.sleep(0.05)

    else:
        glob_BLE_connected = 0
        raise DeviceNotFoundError



def backend_init():

        glob_model['is_init'] = True
        glob_model['CAR_NAME'] = DEFAULT_CAR_NAME
        glob_model['data'] = {}
        glob_model['BLE_Task'] = None


        
def backend_connect():
    glob_model['BLE_Task'] = asyncio.create_task(ble_task(iq, oq, tq))
        
def backend_disconnect():
    global glob_UI_disconnected
    global glob_BLE_connected
    glob_UI_disconnected = 1
    glob_BLE_connected = 1
        
def backend_send_msg():
    message = comm_text_input.value
    backend_enqueue_message(message)
        
def backend_enqueue_message(message):
    if glob_BLE_connected:
        iq.put(message)
        comm_text_input.value = ''
    
def backend_set_car_name(e):
    glob_model['CAR_NAME'] = e.value
    
def backend_joystick_max_speed(e):
    glob_model['joystick_request_speed'] = e.value
    
def backend_joystick_compute(e):
    glob_model['joystick'] = [e.x, e.y, 1]


def backend_joystick_end():
    glob_model['joystick']=[0,0,1]
    joystick_label.text = ''
    cmd_str = backend_joystick_move_cmd()
    backend_enqueue_message(cmd_str)
    glob_model['joystick']=[0,0,0]
    
def backend_move_quick_reverse(e):
    cmd_str = 'car_m_move {} {} {}'.format(-200, -200, 750)
    backend_enqueue_message(cmd_str)
    
def backend_joystick_move_cmd():
    left_speed = 0
    right_speed = 0
    duration = 500
    magnitude = math.sqrt(glob_model['joystick'][0]**2 + glob_model['joystick'][1]**2)
    max_speed = float(glob_model['joystick_car_speed'])
    if (glob_model['joystick'][0] > 0):
        left_speed = int(max_speed * magnitude)
        right_speed = int(max_speed * magnitude * glob_model['joystick'][1])
    else:
        left_speed = int(max_speed * magnitude * glob_model['joystick'][1])
        right_speed = int(max_speed * magnitude)
      
    cmd_str = 'car_m_move {} {} {}'.format(left_speed, right_speed, duration)
    return cmd_str
    
    
def backend_update():
    
    global glob_BLE_connected

    if glob_model['is_init']:

        if (glob_model['BLE_Task'] is not None):
            if (glob_model['BLE_Task'].done()):
                glob_BLE_connected = 0

        
        if (not oq.empty()):
            # log_str = '{:12d}:\t{}'.format(glob_model['data']['time_ms'],
            #                              glob_model['data']['message'])
            log_str = oq.get()
            comm_log.push(log_str)

        if (not tq.empty()):
            glob_model['data'] = tq.get()
            
            cal_plot_datapoint = [
                int(glob_model['data']['time_ms']),
                [
                    int(glob_model['data']['IMU']['calibration']['system_cal']),
                    int(glob_model['data']['IMU']['calibration']['gryo_cal']),
                    int(glob_model['data']['IMU']['calibration']['accel_cal']),
                    int(glob_model['data']['IMU']['calibration']['mag_cal'])
                ]
            ]
            
            glob_model['calibration_plot_data'].put(cal_plot_datapoint)


def backend_medium_update():
    if glob_model['is_init']:
        if glob_model['joystick'][2] == 1:
            if (glob_model['joystick'][0] and glob_model['joystick'][1]):
                cmd_str = backend_joystick_move_cmd()
                joystick_label.text = cmd_str
                backend_enqueue_message(cmd_str)

def backend_slow_update():
    if glob_model['is_init']:
        
        json_view.properties['content'] = {'json' : glob_model['data']}
        json_view.update()
                
        while not glob_model['calibration_plot_data'].empty():
            datapoint = glob_model['calibration_plot_data'].get()
            calibration_plot.options['series'][0]['data'] = (datapoint[1])
                
        backlog_plot.options['series'][0]['data'][0] = oq.qsize()

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

            
def backend_very_slow_update():
    
    # Testing only    
    if TESTING_MODE:
        # backlog_plot.options['series'][0]['data'][0] = np.random.rand() * 200
        calibration_test_data = np.random.rand(4)*3
        calibration_test_data = calibration_test_data.tolist()
        calibration_plot.options['series'][0]['data'] = calibration_test_data
    
    calibration_plot.update()
    backlog_plot.update()



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
ui.timer(2, callback=backend_very_slow_update)


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
        

with ui.right_drawer(top_corner=True, bottom_corner=True).style('background-color: #d7e3f4'):
    ui.markdown('##Robot Status')

    with ui.grid(rows='1fr 1fr').classes('h-full gap-0'):

        with ui.scroll_area().classes('w-full h-full'):
            ui.markdown('###Plots')
            ui.separator()
            
            calibration_plot = ui.echart({
                'xAxis': {'type': 'value', 'min': 0 , 'max': 3},
                'yAxis': {'type': 'category',
                            'data': ['System', 'Gyro', 'Accel', 'Mag'],
                            'inverse': True
                            },
                'legend': {'textStyle': {'color': 'gray'}},
                'series': [
                    {'type': 'bar', 'name': 'IMU Calibration Status', 'data': [0,0,0,0], 'color': '#7DDA58'},
                ],
                'grid': {'containLabel': True, 'left': 0},
                })
            calibration_plot.classes('w-10/12 h-2/12')
            
            
            backlog_plot = ui.echart({
                'xAxis': {'type': 'value', 'min': 0 , 'max': 10},
                'yAxis': {'type': 'category', 'data': ['BL'], 'inverse': True,
                        'nameRotate': 90,},
                'legend': {'textStyle': {'color': 'gray'}},
                'series': [
                    {'type': 'bar', 'name': 'Command Backlog', 'data': [0.1]},
                ],
                'grid': {'containLabel': True, 'left': 0},
                })
            backlog_plot.classes('w-10/12 h-2/12')
        

        with ui.column().classes('w-full'):

            ui.markdown('###Telemetry Data')
            json_view = ui.json_editor({'content': {'json': {}}})
            json_view.classes(('w-fit h-fit'))





    
with ui.card() as comm_window:
    comm_window.tight()
    comm_window.classes('w-11/12')
    with ui.card_section():
        ui.label('Communication Window')
    comm_log = ui.log(max_lines=100)
    comm_log.classes('w-full h-3/12')
    comm_log.style('font-size: 75%;')
    comm_text_input = ui.input(label='Enter commands here:')
    comm_text_input.classes('w-full h-20')
    comm_text_input.on('keydown.enter', backend_send_msg) 
    
ui.separator()  

with ui.grid(columns='2fr 1fr').classes('w-full gap-0'):

    with ui.card() as joystick_card:
        joystick_card.tight()
        joystick_card.classes('w-11/12 h-80')
        with ui.card_section():
            ui.label('Manual control joystock')
        joystick = ui.joystick(color='blue', size=150,
                            on_move=backend_joystick_compute,
                            on_end=backend_joystick_end)
        joystick.classes('w-11/12 h-11/12 bg-slate-300')
        
        with ui.card_section():
            with ui.grid(columns='1fr 2fr').classes('w-full gap-0'):
                joystick_max_speed_input = ui.input(label='Enter max speed here',
                                                    value=DEFAULT_MAUAL_SPEED,
                                                    on_change=backend_joystick_max_speed)
                
                joystick_label = ui.label('')
                
                ui.button('Reverse!', on_click=backend_move_quick_reverse)
        
    with ui.card() as placeholder_card:
        placeholder_card.tight()
        placeholder_card.classes('w-11/12')
        with ui.card_section():
            ui.label("Placeolder")


      


glob_model['is_ui_init'] = True



ui.run(native=True, window_size=(1200, 1000), fullscreen=False, reload=False, port=native.find_open_port())

# To build as exe, run the following in command line (need pyinstaller)
#  nicegui-pack --onefile --windowed --name "RCC" main.py