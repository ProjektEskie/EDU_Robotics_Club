import asyncio
import time

queue = asyncio.Queue()
output_queue = asyncio.Queue()

async def tcp_echo_client(message):
    
    reader, writer = await asyncio.open_connection(
        '192.168.1.16', 8765)
    
    # message = await queue.get()
        
    print(f'Send: {message!r}')
    writer.write(message.encode())
    await writer.drain()
    time_sent = time.monotonic()

    data = await reader.readline()
    data_str = data.decode()
    # print(f'Received: {data.decode()!r}', "size: ", len(data), "bytes")
    time_received = time.monotonic()
    
    await output_queue.put(data_str)

    print('Close the connection')
    writer.close()
    await writer.wait_closed()
    
    await asyncio.sleep(0.2)
    
    
async def arduino_talk_loop():
    
    n_retries = 0
    
    while True:
        if queue.empty():
            queue.put_nowait('query' + '\n')
        
        if n_retries == 0:
            message = queue.get_nowait()

        try:
            await tcp_echo_client(message)
            n_retries = 0
            queue.task_done()
        except ConnectionResetError as e:
            n_exceptions += 1
            n_retries += 1
            print("Connection reset error, attempt: ", n_retries)
            
            if n_retries > 3:
                n_retries = 0
                print("Connection failure, skipping to next")
                queue.task_done()
    

if __name__ == "__main__":

    queue.put_nowait('Test' + '\n')
    asyncio.run(arduino_talk_loop())

    for i in range(0, 100):
        
        if (i%5 == 0):
            queue.put_nowait('Hello {0}'.format(i) + '\n')
            
        time.sleep(1)
        
        while not output_queue.empty():
            print(queue.get_nowait())