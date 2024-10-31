#!/usr/bin/env python3
#Script for BitfocusCompanion to use chatbox of TallyArbiter

#sudo pip3 install "python-socketio[client]<5": This library is used to communicate with a Tally Arbiter server over websockets.
#sudo pip3 install websockets

import sys
import os
import socketio
import json
import argparse
import asyncio
import time
import websockets
import websocket as websocket_c
try:
    import thread
except ImportError:
	import _thread as thread

USERS = set()
serverhost = '0.0.0.0'	#broadcast on all 
serverclient = 'localhost'
port = 1080

messagecmd = ""         #message from Companion to TallyArbiter
messageresponse = ""    #message from Tally listener to Companion
usermessage = ""
deviceId = ""
device_states = []

sio = socketio.Client()

while True:
    try:
        def tally_process(): #TallyArbiter connection process
            @sio.event
            def connect():
                print("Connected to Tally Arbiter server.")
                sio.emit("listenerclient_connect",{"deviceId": "ed34bacd","listenerType": "socket_","canBeReassigned": False,"canBeFlashed": False,"supportsChat": True})

            @sio.event
            def connect_error(data):
                print("Unable to connect to Tally Arbiter server.")

            @sio.event
            def disconnect():
                print("Disconnected from Tally Arbiter server.")

            @sio.event
            def reconnect():
                print("Reconnected to Tally Arbiter server.")

            @sio.on("error")
            def on_error(error):
                print(error)

            @sio.on("device_states")
            def on_device_states(data):
                global device_states
                device_states = data
                #print(data) #for testing

            @sio.on("reassign")
            def on_reassign(oldDeviceId, newDeviceId, gpoGroupId):
                print("Reassigning " + " from DeviceID: " + oldDeviceId + " to DeviceID: " + newDeviceId)
                sio.emit("listener_reassign_object", data=(oldDeviceId, newDeviceId))

            @sio.on("messaging") #receive response from Tally listener
            def on_messaging(messagetype, socketID, message):
                global messageresponse
                global usermessage
                if (message[0] != "#"): #do nothing when command is echoed
                    messageresponse = '{"' + messagetype[:2] + '":"' + message + '"}' #first 2 characters of messagetype, JSON format
                    usermessage = messageresponse #parse to notify_users()

            def server_connect(url):
                try:
                    print("Attempting to connect to Tally Arbiter server: " + url)
                    sio.connect(url)
                    sio.wait()
                except socketio.exceptions.ConnectionError:
                    print("Connection error: retrying in 15 seconds.")
                    time.sleep(15)
                    server_connect(url)
            if __name__ == "__main__":
                print("Tally Arbiter socket Listener Running. Press CTRL-C to exit.")
                try:
                    server_connect("http://127.0.0.1:4455") #local TallyArbiter server
                    #server_connect("http://192.168.178.23:4455")
                except KeyboardInterrupt:
                    print("Exiting Tally Arbiter socket Listener.")
                    exit(0)
                except:
                    print("Unexpected error:", sys.exc_info()[0])
                    exit(0)

        def sendmessage(message):
            sio.emit("messaging", ("Socket", message.strip()))

#process between websocket server and TallyArbiter process
        def process_loop(): #receive message from Companion and send to TallyArbiter chatbox
            global messagecmd
            while True:
                if (messagecmd != ""):
                    sendmessage(messagecmd)
                    print("Message to Tally: " + messagecmd)
                    messagecmd = ""
                time.sleep(0.1)

#websocket client       
        def notify_process():
            def on_message(ws, message):
                print("Client received message: ",message)
                
            def on_error(ws, error):
                print("Client error")
                print(error)

            def on_close(ws):
                print("### Client closed ###")

            def on_open(ws):
                def run(*args):
                    global messageresponse
                    while True:
                        if (messageresponse != ""): #response from tally listener send to websocket server
                            ws.send(messageresponse)
                            messageresponse = ""
                        time.sleep(0.1)
                thread.start_new_thread(run, ())

            if __name__ == "__main__":
                time.sleep(3)   #some delay for starting socketserver first
                websocket_c.enableTrace(False)
                ws = websocket_c.WebSocketApp("ws://" + serverclient + ":" + str(port),
                    on_message = on_message,
                    on_error = on_error,
                    on_close = on_close)
                ws.on_open = on_open
                ws.run_forever()

#websocket server
        def users_event():
            global usermessage
            return usermessage
			
        async def notify_users(): #there are 2 clients, Companion and local websocket client
            if USERS:
                message = users_event()
                for user in USERS:
                    await user.send(message.strip())

        async def register(websocket):
            USERS.add(websocket)
            await websocket.send("welcome, you are connected to the Tally socketserver")
            print("User connected")

        async def unregister(websocket):
            USERS.remove(websocket)
            print("User disconneted")

        async def Tally(websocket, path):
            global messagecmd
            await register(websocket)
            try:
                async for message in websocket:
                    try:
                        print("Server received message: " + message)
                        if (message[0] == "#" or message[0] != "{"):
                            messagecmd = message # for process_loop(), send to TallyArbiter chatbox, except response
                        else:
                            await notify_users()
                    except:
                        await websocket.send("Socket error: 13 – Wrong message type")
            finally:
                await unregister(websocket)

        thread.start_new_thread(tally_process, ()) #TallyArbiter connection
        thread.start_new_thread(notify_process, ()) #websocket client
        thread.start_new_thread(process_loop, ())  
        start_server = websockets.serve(Tally, serverhost, port) #websocket server
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()
        

    except Exception:
        print("Connection error")
        time.sleep(10)
		

