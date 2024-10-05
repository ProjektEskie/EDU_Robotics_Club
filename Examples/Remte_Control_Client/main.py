import threading
import time
import socket
import queue
import json
from nicegui import app, ui, Client
import ipaddress

iq = queue.SimpleQueue()
oq = queue.SimpleQueue()

DEFAULT_IP = '192.168.1.16'

glob_model = {}
glob_model['is_init'] = False
glob_model['is_ui_init'] = False
glob_model['is_connected_chip'] = None
glob_model['is_disconnected_chip'] = None
glob_init_semaphore = threading.Semaphore()
glob_UI_disconnected = 0

def thread_arduino_comms_loop(input_queue, output_queue):
    IP = glob_model['IP']
    PORT = 8765
    
    while True:
        global glob_UI_disconnected
        if glob_UI_disconnected:
            print("UI disconnect detected, ending comm thread")
            break
        
        if input_queue.empty():
            input_queue.put(b'query\n')
            
        message = input_queue.get()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(3)
            s.connect((IP, PORT))
            s.sendall(message)
            try:
                data = s.recv(1024)
            except ConnectionResetError as e:
                # this error is generated if the arduino stops sending, no need for action
                
                pass
            
            except TimeoutError as e:
                print(e)
                break
            
            json_data = json.loads(data)
        
        output_queue.put(json_data)
            
        time.sleep(0.1)
        
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
    iq.put(message.encode())
    comm_text_input.value = ''
    
def backend_ip_validation(value):
    try:
        ipaddress.ip_address(value)
    except ValueError:
        return "Invalid IP address"
    
def backend_setip(e):
    glob_model['IP'] = e.value
    
    
def backend_update():
    
    if glob_model['is_init']:
        
        if (not oq.empty()):
            glob_model['data'] = oq.get()
            
            if len(glob_model['data']['message']) > 0:
                log_str = '{:12d}:\t{}'.format(glob_model['data']['time_ms'],
                                         glob_model['data']['message'])
                comm_log.push(log_str)
            
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
        
app.native.start_args['debug'] = False
app.native.settings['ALLOW_DOWNLOADS'] = True
app.on_disconnect(backend_disconnect)


ui.timer(0, callback=backend_init, once=True)
ui.timer(0.1, callback=backend_update)
ui.timer(0.5, callback=backend_slow_update)


ui.separator()


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
    

ui.separator()

with ui.row().classes('w-full h-4/12'):
    comm_window = ui.card()
    comm_window.classes('w-6/12')
    with comm_window.tight():
        with ui.card_section():
            ui.label('Communication Window')
        comm_log = ui.log(max_lines=100)
        comm_log.classes('w-full h-4/12')
        comm_log.style('font-size: 75%;')
        comm_text_input = ui.input(label='Enter commands here:')
        comm_text_input.classes('w-full h-10')
        comm_text_input.on('keydown.enter', backend_send_msg) 
        
    ui.space()
    
    with ui.card() as json_card:
        json_card.tight()
        json_card.classes('w-fit')
        with ui.card_section():
            ui.label('Raw Data')
        json_view = ui.json_editor({'content': {'json': {}}})
        
ui.separator()  


glob_model['is_ui_init'] = True
ui.run(native=True, window_size=(1000, 1000), fullscreen=False, reload=False)