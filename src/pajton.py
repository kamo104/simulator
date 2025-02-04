#!/usr/bin/env python3

import asyncio
import websockets
import json

async def send_message():
    uri = "ws://127.0.0.1:42069"  # Replace with your WebSocket server URL
    message = {
        "aircrafts": [
            {
                "id": -1986102845,
                "position": {
                    "altitude": 982.5443825945306,
                    "latitude": 52.37139765627405,
                    "longitude": 16.27626783819914
                },
                "squawk": "2211",
                "velocity": {
                    "direction": -3.003891706466675,
                    "value": 115.36572265625
                }
            }
        ],
        "type": "positions"
    }
    
    async with websockets.connect(uri) as websocket:
        msg = await websocket.recv()
        await websocket.send(json.dumps(message))
        print("Message sent")
        response = await websocket.recv()
        print("Received response:", response)

asyncio.run(send_message())
