import threading
import time
import socket
import queue
import json
from nicegui import app, ui, native
import ipaddress
import math
import numpy as np
import plotly.graph_objects as go


iq = queue.SimpleQueue()
oq = queue.SimpleQueue()

DEFAULT_IP = '192.168.1.'
DEFAULT_MAUAL_SPEED='180'

glob_model = {}
glob_model['is_init'] = False
glob_model['is_ui_init'] = False
glob_model['is_connected_chip'] = None
glob_model['is_disconnected_chip'] = None
glob_model['joystick'] = [0,0,0]
glob_model['joystick_request_speed'] = [0,0]
glob_model['joystick_car_speed'] = DEFAULT_MAUAL_SPEED
glob_model['calibration_plot_data'] = queue.SimpleQueue()
glob_model['ping_plot_data'] = queue.SimpleQueue()
glob_init_semaphore = threading.Semaphore()
glob_UI_disconnected = 0

def thread_arduino_comms_loop(input_queue, output_queue):
    IP = glob_model['IP']
    PORT = 8765
    
    is_connected = False
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    while True:
        global glob_UI_disconnected
        if glob_UI_disconnected:
            print("UI disconnect detected, ending comm thread")
            if (is_connected):
                s.close()
            break
        
        if (not is_connected):
            s.settimeout(2)
            s.connect((IP, PORT))
            is_connected = True
        
        if input_queue.empty():
            input_queue.put(b'query\n')
            
        message = input_queue.get()

        s.sendall(message)
        try:
            data = s.recv(1024)
        except ConnectionResetError as e:
            # this error is generated if the arduino stops sending, no need for action
            is_connected = False
            pass
        
        except TimeoutError as e:
            print(e)
            break
        
        json_data = json.loads(data)
        
        output_queue.put(json_data)
            
        time.sleep(0.05)
        
    return

# @ui.page('/')
# async def index(client: Client):
#     print('preparing')
#     await client.connected()
#     print('connected')
#     await client.disconnected()
#     print('disconnected')
    
        
def backend_init():
        arduino_comms_thread = threading.Thread(target=thread_arduino_comms_loop, args=[iq, oq])
        glob_model['comms_thread'] = arduino_comms_thread
        glob_model['is_init'] = True
        glob_model['IP'] = '192.168.1.16'
        glob_model['data'] = {}
        
        print("hi", threading.current_thread())

        
def backend_connect():
    try:
        glob_model['comms_thread'].start()
    except RuntimeError:
        arduino_comms_thread = threading.Thread(target=thread_arduino_comms_loop, args=[iq, oq])
        glob_model['comms_thread'] = arduino_comms_thread
        glob_model['comms_thread'].start()
        
def backend_disconnect():
    if (glob_model['comms_thread'].is_alive()):
        global glob_UI_disconnected
        glob_UI_disconnected = 1
        
def backend_send_msg():
    message = comm_text_input.value + '\n'
    backend_enqueue_message(message)
        
def backend_enqueue_message(message):
    if (glob_model['comms_thread'].is_alive()):
        iq.put(message.encode())
        comm_text_input.value = ''
    
def backend_ip_validation(value):
    try:
        ipaddress.ip_address(value)
    except ValueError:
        return "Invalid IP address"
    
def backend_setip(e):
    glob_model['IP'] = e.value
    
def backend_joystick_max_speed(e):
    glob_model['joystick_request_speed'] = e.value
    
def backend_joystick_compute(e):
    glob_model['joystick'] = [e.x, e.y, 1]


def backend_joystick_end():
    glob_model['joystick']=[0,0,1]
    joystick_label.text = ''
    cmd_str = backend_joystick_move_cmd()
    backend_enqueue_message(cmd_str + '\n')
    glob_model['joystick']=[0,0,0]
    
def backend_move_quick_reverse(e):
    cmd_str = 'car_m_move {} {} {}'.format(-200, -200, 750)
    backend_enqueue_message(cmd_str + '\n')
    
def backend_joystick_move_cmd():
    left_speed = 0
    right_speed = 0
    duration = 1500
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
    
    if glob_model['is_init']:
        
        if (not oq.empty()):
            glob_model['data'] = oq.get()
            
            if len(glob_model['data']['message']) > 0:
                log_str = '{:12d}:\t{}'.format(glob_model['data']['time_ms'],
                                         glob_model['data']['message'])
                comm_log.push(log_str)
                
            cal_plot_datapoint = [
                int(glob_model['data']['time_ms']),
                [
                    int(glob_model['data']['calibration']['system_cal']),
                    int(glob_model['data']['calibration']['gryo_cal']),
                    int(glob_model['data']['calibration']['accel_cal']),
                    int(glob_model['data']['calibration']['mag_cal'])
                ]
            ]
            
            ping_plot_datapoint = [
                int(glob_model['data']['time_ms']),
                int(glob_model['data']['t_last'])
            ]
            
            glob_model['calibration_plot_data'].put(cal_plot_datapoint)
            glob_model['ping_plot_data'].put(ping_plot_datapoint)
            
        if glob_model['comms_thread'].is_alive():
            is_disconnected_chip.set_visibility(False)
            is_connected_chip.set_visibility(True)
        else:
            is_disconnected_chip.set_visibility(True)
            is_connected_chip.set_visibility(False)

        
def backend_slow_update():
    if glob_model['is_init']:
        
        json_view.properties['content'] = {'json' : glob_model['data']}
        json_view.update()
        
        if glob_model['joystick'][2] == 1:
            if (glob_model['joystick'][0] and glob_model['joystick'][1]):
                cmd_str = backend_joystick_move_cmd()
                joystick_label.text = cmd_str
                backend_enqueue_message(cmd_str + '\n')
                
        while not glob_model['calibration_plot_data'].empty():
            datapoint = glob_model['calibration_plot_data'].get()
            calibration_plot.push(datapoint)
                
        while not glob_model['ping_plot_data'].empty():
            datapoint = glob_model['ping_plot_data'].get()
            ping_plot.options['series'][0]['data'][0] = datapoint[1]

            
def backend_very_slow_update():
    
    # Testing only    
    # ping_plot.options['series'][0]['data'][0] = np.random.rand() * 200
    
    ping_plot.update()

    
app.native.start_args['debug'] = False
app.native.settings['ALLOW_DOWNLOADS'] = True
app.on_disconnect(backend_disconnect)

ui.timer(0, callback=backend_init, once=True)
ui.timer(0.1, callback=backend_update)
ui.timer(0.5, callback=backend_slow_update)
ui.timer(2, callback=backend_very_slow_update)


with ui.header(elevated=True).style('background-color: #3874c8').classes('items-center justify-between'):


    with ui.row().classes('w-full'):
        
        ui.markdown('#Robot Car Control')

        
        ui.space()
        
        ip_textbox = ui.input(label='Enter IP address to connect',
                                value=DEFAULT_IP,
                                on_change=backend_setip,
                                validation=backend_ip_validation)
        
        ip_textbox.on('keydown.enter', backend_connect) 
        

        is_disconnected_chip = ui.chip("Offline",
                                    icon='blocked',
                                    color='red')
        is_disconnected_chip.set_enabled(False)
        is_connected_chip = ui.chip('Connected',
                                    icon='thumb_up',
                                    color='green')
        is_connected_chip.set_enabled(False)
        is_connected_chip.set_visibility(False)
        

with ui.grid(columns='2fr 1fr').classes('w-full gap-0'):
    
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
            

        
ui.separator()  

with ui.grid(columns='2fr 1fr').classes('w-full gap-0'):
    
    with ui.card() as cal_status_card:
        cal_status_card.tight()
        cal_status_card.classes('w-11/12 h-3/12')
        with ui.card_section():
            ui.label("IMU Calibration status")
        with ui.card_section():
            calibration_plot = ui.line_plot(n=4, limit=100, figsize=(8, 3), update_every=1) \
                .with_legend(
                    ['System', 'Gyro', 'Accel', 'Mag'],
                    loc='lower left', ncol=2)

        
    with ui.card() as json_card:
        json_card.tight()
        json_card.classes('w-11/12')
        with ui.card_section():
            ui.label('Raw Data')
        json_view = ui.json_editor({'content': {'json': {}}})
        json_view.classes(('w-fit'))

ui.separator()  
with ui.grid(columns='2fr 1fr').classes('w-full gap-0'):

    with ui.card() as ping_plot_card:
        ping_plot_card.tight()
        ping_plot_card.classes('w-11/12')

        ping_plot = ui.echart({
            'xAxis': {'type': 'value', 'min': 0 , 'max': 200},
            'yAxis': {'type': 'category', 'data': ['Ping'], 'inverse': True},
            'series': [
                {'type': 'bar', 'name': 'RoundTrip', 'data': [0.1]},
            ],
        })

        
    with ui.card() as placeholder_card:
        placeholder_card.tight()
        placeholder_card.classes('w-11/12')
        with ui.card_section():
            ui.label("Placeolder")

glob_model['is_ui_init'] = True



ui.run(native=True, window_size=(1200, 1000), fullscreen=False, reload=False, port=native.find_open_port())

# To build as exe, run the following in command line (need pyinstaller)
#  nicegui-pack --onefile --windowed --name "RCC" main.py