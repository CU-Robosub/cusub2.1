#!/usr/bin/python3

import argparse
import csv
import datetime
import json
import socket
import threading

DEFAULT_IP = "192.168.194.95"
DEFAULT_PORT = 16171


class datareader:
    def __init__(self):
        self.msg = None
        self.dvl_socket = None
        self.ip = DEFAULT_PORT
        print(self.reset_deadreckoning())
    
    def is_connected(self):
        return (self.dvl_socket is not ConnectionRefusedError or self.dvl_socket is not None)

    def connect_dvl(self):
        try:
            dvl_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            dvl_socket.connect((self.ip, DEFAULT_PORT))
            self.dvl_socket = dvl_socket
            return dvl_socket
        except ConnectionRefusedError:
            return ConnectionRefusedError
    
    def _type(self, message_type):
        if message_type == "velocity":
            return "velocity"
        return "position_local"

    def read_data(self, message_type, time_format="%Y-%m-%d %H:%M:%S"):
        message_type = self._type(message_type)
        buffer_size = 4096
        message = ""
        while True:
            buffer = self.dvl_socket.recv(buffer_size).decode()
            if not buffer:
                continue
            message_parts = buffer.split("\r\n")
            if len(message_parts) == 1:
                message += message_parts[0]
                continue
            for message_part in message_parts[:-1]:
                message = message + message_part
                self._handle(message_type, message, time_format)
                message = ""
                return self.getMessage()
            if message_parts[-1]:
                message = message_parts[-1]
                return self.getMessage()


    def _format_timestamp(self, timestamp, time_format):
        return datetime.datetime.strftime(
            datetime.datetime.fromtimestamp(timestamp),
            time_format)

    def _format_timestamps(self, message_type, message, time_format):
        message["log_time"] = self._format_timestamp(
            message["log_time"] / 1e6,
            time_format)
        if message_type == "velocity":
            try:
                message["time_of_validity"] = self._format_timestamp(
                    message["time_of_validity"] / 1e6,
                    time_format)
                message["time_of_transmission"] = self._format_timestamp(
                    message["time_of_transmission"] / 1e6,
                    time_format)
            except KeyError:
                pass
        else:
            message["ts"] = self._format_timestamp(message["ts"], time_format)

    def _handle(self, message_type, message, time_format):
        """Handle a message from the DVL. Set self.message to the message."""
        if not message:
            return
        
        try:
            report = json.loads(message)
        except json.decoder.JSONDecodeError:
            print("Could not parse to JSON: " + message)
            return
        
        print(report["type"])
        if report["type"] != message_type:
            return
        report["log_time"] = int(datetime.datetime.utcnow().timestamp() * 1e6)
        if time_format:
            self._format_timestamps(message_type, report, time_format)
        self.msg = (json.dumps(report)) # the key function to return all data
    
    def getMessage(self):
        return self.msg
    
    def reset_deadreckoning(self):
        cmd = {"command": "reset_dead_reckoning"}
        self.dvl_socket.send(json.dumps(cmd).encode())
        
        response = self.dvl_socket.recv(4096).decode()
        return json.dumps(response)
    
    def calibrate_gyro(self):
        cmd = {"command": "calibrate gyro"}
        self.dvl_socket.send(json.dumps(cmd).encode())
        
        response = self.dvl_socket.recv(4096).decode()
        return json.dumps(response)