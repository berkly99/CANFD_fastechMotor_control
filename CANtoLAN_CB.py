#*************************************************************************************************************************************
#** Notification before use																											**
#*************************************************************************************************************************************
#** Depending on the type of product you are using, the definitions of Parameter, IO Logic, AxisStatus, etc. may be different.		**
#** This example is based on Ezi-SERVO2, so please apply the appropriate value depending on the product you are using.				**
#*************************************************************************************************************************************
#** ex)	FM_EZISERVO2_PARAM			// Parameter enum when using Ezi-SERVO2						 									**
#**		FM_EZIMOTIONLINK2_PARAM		// Parameter enum when using Ezi-MOTIONLINK2													**
#*************************************************************************************************************************************

import os
import time
import can
import threading
import struct
import cantools
from pymodbus.client import ModbusSerialClient    # pymodbus MODBUS protocol



os.system('sudo ip link set can0 up type can bitrate 500000 dbitrate 2000000 sample-point 0.8 dsample-point 0.8 restart-ms 100 berr-reporting on fd on')

from FAS_EziMOTIONPlusE import *
from MOTION_DEFINE import *
from ReturnCodes_Define import *
from MOTION_EziSERVO2_DEFINE import *

TCP = 0
UDP = 1

# Variable to store the previous state (8 bits)
previous_state = 0

dbc_file = '/home/pi/Desktop/PE/Example/28.CANtoLAN/CANFD_250714_CB.dbc'
db = cantools.database.load_file(dbc_file)

motor_indices = set()

for msg in db.messages:
    for signal in msg.signals:
        name = signal.name
        if name.startswith("Servo_"):
            # "Servo_" 뒤에 오는 숫자 추출
            suffix = name[6:]  # "Servo_" 길이 6
            if suffix.isdigit():
                motor_indices.add(int(suffix))

if motor_indices:
    max_motor = max(motor_indices)
    print(f"Detected Servo indices: {sorted(motor_indices)}")
    print(f"Estimated motor count: {max_motor}")
else:
    print("No 'Servo_n' keys found.")


ser = ModbusSerialClient(      # modbus protocol
    port='/dev/ttyAMA0',
    baudrate=115200,           # baudrate  115200
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=1
)

class EdgeTriggerController:
    def __init__(self, relay_count=8):
        # Store previous state of all signals, including relays
        self.prev_states = {}
        self.relay_count = relay_count

    def process_message(self, decoded_message):
        # 1. Relay control (Relay_1 ~ Relay_8, ON/OFF by edge)
        for i in range(1, self.relay_count + 1):
            key = f"Relay_{i}"
            current = decoded_message.get(key)
            prev = self.prev_states.get(key)
            if prev is None:
                self.prev_states[key] = current
                continue

            # RISING edge: LOW -> HIGH (ON)
            if prev == "LOW" and current == "HIGH":
                response = ser.write_coil(i, 1, slave=1)
                if response.isError():
                    print(f"{key} ON Error: {response.exception_code}")
                else:
                    print(f"{key} ON Write successful")
            # FALLING edge: HIGH -> LOW (OFF)
            elif prev == "HIGH" and current == "LOW":
                response = ser.write_coil(i, 0, slave=1)
                if response.isError():
                    print(f"{key} OFF Error: {response.exception_code}")
                else:
                    print(f"{key} OFF Write successful")

            self.prev_states[key] = current

        # 2. JOG control (JOG_Plus_1, JOG_Minus_1, JOG_Plus_2, JOG_Minus_2, unified START/STOP)
        for motor_id in [1, 2]:
            for dir_name, dir_value in [("Plus", 1), ("Minus", 0)]:
                jog_key = f"JOG_{dir_name}_{motor_id}"  # e.g., JOG_Plus_1
                current = decoded_message.get(jog_key)
                prev = self.prev_states.get(jog_key)
                if prev is None:
                    self.prev_states[jog_key] = current
                    continue

                # RISING edge: LOW -> HIGH (START JOG)
                if prev == "LOW" and current == "HIGH":
                    if not JogMove(motor_id, dir_value):
                        print(f"Jog {dir_name} {motor_id} Start Fail...")
                # FALLING edge: HIGH -> LOW (STOP JOG)
                elif prev == "HIGH" and current == "LOW":
                    if not Stop(motor_id):
                        print(f"Jog {dir_name} {motor_id} Stop Fail...")

                self.prev_states[jog_key] = current

        # 3. Servo ON/OFF (Servo_1, Servo_2, unified ON/OFF)
        for motor_id in [1, 2]:
            key = f"Servo_{motor_id}"
            current = decoded_message.get(key)
            prev = self.prev_states.get(key)
            if prev is None:
                self.prev_states[key] = current
                continue

            # RISING edge: LOW -> HIGH (Servo ON)
            if prev == "LOW" and current == "HIGH":
                if not SetServoOn(motor_id):
                    print(f"Servo{motor_id} On Fail...")
            # FALLING edge: HIGH -> LOW (Servo OFF)
            elif prev == "HIGH" and current == "LOW":
                if not SetServoOff(motor_id):
                    print(f"Servo{motor_id} Off Fail...")

            self.prev_states[key] = current

        # 4. Other motor commands (Stop, Em_Stop, ORG, AlarmReset, Abs/Inc/Dec Move)
        signals = [
            ("Stop_1", Stop, 1, None),
            ("Stop_2", Stop, 2, None),
            ("Em_Stop_1", Emergency_Stop, 1, None),
            ("Em_Stop_2", Emergency_Stop, 2, None),
            ("ORG_Search_1", OriginSearch, 1, None),
            ("ORG_Search_2", OriginSearch, 2, None),
            ("Alarm_Reset_1", AlarmReset, 1, None),
            ("Alarm_Reset_2", AlarmReset, 2, None),
            ("Abs_Pos_Move_1", AbsolutePositionMove, 1, "Cmd_Pos_1"),
            ("Abs_Pos_Move_2", AbsolutePositionMove, 2, "Cmd_Pos_2"),
            ("Pos_Inc_Move_1", PositionIncreaseMove, 1, "Cmd_Pos_1"),
            ("Pos_Inc_Move_2", PositionIncreaseMove, 2, "Cmd_Pos_2"),
            ("Pos_Dec_Move_1", PositionDecreaseMove, 1, "Cmd_Pos_1"),
            ("Pos_Dec_Move_2", PositionDecreaseMove, 2, "Cmd_Pos_2"),
        ]

        for signal_name, func, motor_id, param_key in signals:
            current = decoded_message.get(signal_name)
            prev = self.prev_states.get(signal_name)
            if prev is None:
                self.prev_states[signal_name] = current
                continue

            # RISING edge: LOW -> HIGH
            if prev == "LOW" and current == "HIGH":
                if param_key:
                    param_value = decoded_message.get(param_key)
                    if not func(motor_id, param_value):
                        print(f"{signal_name} Fail...")
                else:
                    if not func(motor_id):
                        print(f"{signal_name} Fail...")

            self.prev_states[signal_name] = current

# Example usage:
# edge_controller = EdgeTriggerController()
# edge_controller.process_message(decoded_message)




def Connect(nCommType: int, nBdID: int, n4IP: int) -> bool:
    byIP = [192, 168, 0, n4IP]  # IP: 192.168.0.99~100
    bSuccess = True

    # Connection
    if nCommType == TCP:  # TCP Connection
        if FAS_ConnectTCP(byIP[0], byIP[1], byIP[2], byIP[3], nBdID) == 0:
            print("TCP Connection Fail!")
            bSuccess = False
    elif nCommType == UDP:  # UDP Connection
        if FAS_Connect(byIP[0], byIP[1], byIP[2], byIP[3], nBdID) == 0:
            print("UDP Connection Fail!")
            bSuccess = False
    else:
        print("Wrong communication type.")
        bSuccess = False

    if bSuccess:
        print(f"Connected successfully. 192.168.0.{n4IP}")

    return bSuccess

# Device Connect
"""
if not Connect(TCP, 1, 98):     # Motor 1
    input("Press Enter to exit...")
    exit(1)
if not Connect(TCP, 2, 99):     # Motor 2
    input("Press Enter to exit...")
    exit(1)
"""
for motor_id in range(1, max_motor + 1):
    port_num = 97 + motor_id  # 기존 코드 스타일에 맞춰 포트 번호 설정 (1->98, 2->99, ...)
    if not Connect(TCP, motor_id, port_num):
        input(f"Motor {motor_id} connection failed. Press Enter to exit...")
        exit(1)


def SetParameter(nBdID: int) -> tuple[bool, dict]:

	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	#~~ In this function,												~~
	#~~ please modify the value depending on the product you are using.	~~
	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    # Check The Axis Start Speed Parameter Status
    status_result, lParamVal = FAS_GetParameter(nBdID, SERVO2_ORGSPEED)
    if status_result != FMM_OK:
        print("Function(FAS_GetParameter) was failed.")
        return False, {}
    else:
        print("Load Parameter[Before] : Start Speed = %d[pps]" % lParamVal)

    print(
        "-----------------------------------------------------------------------------%d ID", {nBdID}
    )

    status_result, dwAxisStatus = FAS_GetAxisStatus(nBdID)
    if status_result != FMM_OK:
        print("Function(FAS_GetAxisStatus) was failed.")
        return False, {}
    else:
        print(f"Load AxisStatus : Lamp = {dwAxisStatus:#010x}")

    print(
        "-----------------------------------------------------------------------------"
    )

    if (0x00100000 & dwAxisStatus):
        print("Servo ON")
    else:
        print("Servo OFF")


    status_result, lCmdPos, lActPos, lPosErr, lActVel, wPosItemNo \
        = FAS_GetMotionStatus(nBdID)
    if status_result != FMM_OK:
        print("Function(FAS_GetMotionStatus) was failed.")
        return False
    else:
        ##print(f"Load MotionStatus = lCmdPos: {lCmdPos}")
        print("Load MotionStatus = lCmdPos: %d " % lCmdPos)
        ##print(f"Load MotionStatus = lCmdPos: {lCmdPos:#010x}")
        ##print(f"Load MotionStatus = lCmdPos: {to_signed(lCmdPos):#010x}")
        ##print("Load MotionStatus = lActPos: %d " % to_signed(lActPos))
        ##print("Load MotionStatus = lPosErr: %d " % lPosErr)
        ##print("Load MotionStatus = lActVel: %d " % lActVel)
        ##print("Load MotionStatus = wPosItemNo: %d " % to_signed(wPosItemNo))
        ##print("lCmdPos type:", type(lCmdPos))


    print(
        "--2222222222---------"
    )

    return True, {
            "dwAxisStatus": dwAxisStatus,
            "lCmdPos": lCmdPos,
            "lActPos": lActPos,
            "lPosErr": lPosErr,
            "lActVel": lActVel,
            "wPosItemNo": wPosItemNo
        }

def send_can_message():
    arb_id = 0x601
    data = [0] * 16
    for i in range(2):
            
        success, params = SetParameter(i+1)
        if success:
            dwAxisStatus_bytes = struct.pack('>I', params['dwAxisStatus'])
            data[0+(8*i):4+(8*i)] = dwAxisStatus_bytes
            lCmdPos_bytes = struct.pack('<i', params['lCmdPos'])
            data[4+(8*i):8+(8*i)] = lCmdPos_bytes
        else:
            print(f"Failed to set parameter with ID: {i+1}")

        msg = can.Message(
                    arbitration_id=arb_id,
                    data=data,
                    is_extended_id=False,
                    is_fd=True,
                    bitrate_switch=True
                )

    task = bus.send_periodic(msg, 0.1)

    while True:
        arb_id = 0x601
        data = [0] * 16
        for i in range(2):

            #start_ns = time.perf_counter_ns()
            success, params = SetParameter(i+1)
            #end_ns = time.perf_counter_ns()

            #elapsed_us = (end_ns - start_ns) // 1000

            #print(f"write_coil 처리 시간: {elapsed_us} μs")

            if success:
                dwAxisStatus_bytes = struct.pack('>I', params['dwAxisStatus'])
                data[0+(8*i):4+(8*i)] = dwAxisStatus_bytes
                lCmdPos_bytes = struct.pack('<i', params['lCmdPos'])
                data[4+(8*i):8+(8*i)] = lCmdPos_bytes
            else:
                print(f"Failed to set parameter with ID: {i+1}")

            msg = can.Message(
                        arbitration_id=arb_id,
                        data=data,
                        is_extended_id=True,
                        is_fd=True,
                        bitrate_switch=True
                    )
            task.modify_data(msg)

        time.sleep(3)
"""
            try:
                bus.send(msg)
                print("Message sent successfully!")
            except can.CanError as e:
                print(f"Failed to send message: {e}")
"""
        


# receive CAN data, receive data print
def receive_can_message():
    global previous_state
    edge_controller = EdgeTriggerController()
# edge_controller.process_message(decoded_message)
    edge_controller = EdgeTriggerController()
    while True:
        """
        try:
            msg = bus.recv(timeout=1.0)  # Message receive ( 1 second wait )

            if msg is not None:
                print(f"Message received: ID={hex(msg.arbitration_id)}, Data={msg.data}, FD={msg.is_fd}, BRS={msg.bitrate_switch}")

                # Check all bits of the first byte
                current_state = msg.data[0]

                # Check each bit for changes
                for bit in range(8):
                    current_bit = (current_state >> bit) & 1
                    previous_bit = (previous_state >> bit) & 1

                    if current_bit == 1 and previous_bit == 0:
                        getattr(rising, f'rising_function_{bit}')()
                    elif current_bit == 0 and previous_bit == 1:
                        getattr(falling, f'falling_function_{bit}')()

                continue
            """
        message = bus.recv()
        if message.arbitration_id == 0x600:
            #response = ser.write_coil(0, 1, slave=1)  # ON

            try:
                decoded_message = db.decode_message(message.arbitration_id, message.data)

# DBC file print code list
                """
                print("Decoded message:")
                for key, value in decoded_message.items():
                    print(f"{key}: {value} (type: {type(value)})")  
                print("------------------------")
                """

# Relay ON OFF
                edge_controller.process_message(decoded_message) # relay 1~8

# Stop 1 Motor 2 Motor   
# Emergency Stop 1 Motor 2 Motor
# ORG Search 1 Motor 2 Motor
# Alarm Reset
# Absolute Position Move
# Absolute Increase Move
# Absolute Decrease Move
# JOG JOG 1 Motor
# JOG JOG 2 Motor
# Servo ON OFF
            except Exception as e:
                print(f"Failed to receive message: {e}")



##def main():
       
    ##start_time = time.time()
    
    
    ##end_time = time.time()

    ##elapsed_time = (end_time - start_time) * 1000  # ms change
    ##print(f"Operating time: {elapsed_time:.2f} ms")

def SetServoOn(nBdID: int) -> bool:

	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	#~~ In this function,												~~
	#~~ please modify the value depending on the product you are using.	~~
	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    # Check Drive's Servo Status

    # if ServoOnFlagBit is OFF("0"), switch to ON("1")
    status_result, axis_status = FAS_GetAxisStatus(nBdID)
    if status_result != FMM_OK:
        print("Function(FAS_GetAxisStatus) was failed.")
        return False

    if (axis_status & EZISERVO2_AXISSTATUS.FFLAG_SERVOON) == 0:
        if FAS_ServoEnable(nBdID, 1) != FMM_OK:
            print("Function(FAS_ServoEnable) was failed.")
            return False

        while (
            axis_status & EZISERVO2_AXISSTATUS.FFLAG_SERVOON
        ) == 0:  # Wait until FFLAG_SERVOON is ON
            time.sleep(0.001)

            status_result, axis_status = FAS_GetAxisStatus(nBdID)
            if status_result != FMM_OK:
                print("Function(FAS_GetAxisStatus) was failed.")
                return False

            if (axis_status & EZISERVO2_AXISSTATUS.FFLAG_SERVOON) != 0:
                print("Servo ON")

    else:
        print("Servo is already ON")

    return True

def SetServoOff(nBdID: int) -> bool:
    # Check Drive's Servo Status
    status_result, axis_status = FAS_GetAxisStatus(nBdID)
    if status_result != FMM_OK:
        print("Function(FAS_GetAxisStatus) was failed.")
        return False

    # ON ���¸� OFF ��ȣ�� ��
    if (axis_status & EZISERVO2_AXISSTATUS.FFLAG_SERVOON) != 0:
        if FAS_ServoEnable(nBdID, 0) != FMM_OK:
            print("Function(FAS_ServoEnable) was failed.")
            return False

        while (axis_status & EZISERVO2_AXISSTATUS.FFLAG_SERVOON) != 0:  # OFF
            time.sleep(0.001)
            status_result, axis_status = FAS_GetAxisStatus(nBdID)
            if status_result != FMM_OK:
                print("Function(FAS_GetAxisStatus) was failed.")
                return False

            if (axis_status & EZISERVO2_AXISSTATUS.FFLAG_SERVOON) == 0:
                print("Servo OFF")

    else:
        print("Servo is already OFF")

    return True

def JogMove(nBdID: int, nDir: int) -> bool:

	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	#~~ In this function,												~~
	#~~ please modify the value depending on the product you are using.	~~
	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    # Jogmode Move
    nTargetVeloc = 5000
    nDirect = nDir

    print("----JOG Start---")
    if FAS_MoveVelocity(nBdID, nTargetVeloc, nDirect) != FMM_OK:
        print("Function(FAS_MoveVelocity) was failed.")
        return False
    return True

def Stop(nBdID: int) -> bool:
    if FAS_MoveStop(nBdID) != FMM_OK:
        print("Function(FAS_MoveStop) was failed.")
        return False
    return True

def Emergency_Stop(nBdID: int) -> bool:
    if Emergency_Stop(nBdID) != FMM_OK:
        print("Function(FAS_MoveStop) was failed.")
        return False
    return True

def OriginSearch(nBdID: int) -> bool:

	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	#~~ In this function,												~~
	#~~ please modify the value depending on the product you are using.	~~
	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    # Act Origin Search Function
    print("---------------------------")

    # Origin Search Start
    if FAS_MoveOriginSingleAxis(nBdID) != FMM_OK:
        print("Function(FAS_MoveOriginSingleAxis) was failed.")
        return False

    # Check the Axis status until OriginReturning value is released.

    while True:
        time.sleep(0.001)

        status_result, axis_status = FAS_GetAxisStatus(nBdID)
        if status_result != FMM_OK:
            print("Function(FAS_GetAxisStatus) was failed.")
            return False

        if not (axis_status & EZISERVO2_AXISSTATUS.FFLAG_ORIGINRETURNING):
            break

    if axis_status & EZISERVO2_AXISSTATUS.FFLAG_ORIGINRETOK:
        print("Origin Search Success!")
        return True
    else:
        print("Origin Search Fail!")
        return False
    
def AlarmReset(nBdID: int) -> bool:
    if FAS_ServoAlarmReset(nBdID) != FMM_OK:
        print("Function(FAS_MoveStop) was failed.")
        return False
    return True

def AbsolutePositionMove(nBdID: int, nAbsPos: int) -> bool:

	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	#~~ In this function,												~~
	#~~ please modify the value depending on the product you are using.	~~
	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    # Jogmode Move
    lVelocity = 5000
    print(nAbsPos)

    print("----JOG Start---")
    if FAS_MoveSingleAxisAbsPos(nBdID, nAbsPos, lVelocity) != FMM_OK:
        print("Function(FAS_MoveVelocity) was failed.")
        return False
    return True

def PositionIncreaseMove(nBdID: int, nAbsPos: int) -> bool:

	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	#~~ In this function,												~~
	#~~ please modify the value depending on the product you are using.	~~
	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    # Jogmode Move
    lVelocity = 5000
    print(nAbsPos)
    lAbsPos = abs(nAbsPos)
    print(lAbsPos)
    print("----JOG Start---")
    if FAS_MoveSingleAxisIncPos(nBdID, lAbsPos, lVelocity) != FMM_OK:
        print("Function(FAS_MoveVelocity) was failed.")
        return False
    return True

def PositionDecreaseMove(nBdID: int, nAbsPos: int) -> bool:

	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	#~~ In this function,												~~
	#~~ please modify the value depending on the product you are using.	~~
	#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    # Jogmode Move
    lVelocity = 5000
    lAbsPos = -abs(nAbsPos)   
    print("----JOG Start---")
    if FAS_MoveSingleAxisIncPos(nBdID, lAbsPos, lVelocity) != FMM_OK:
        print("Function(FAS_MoveVelocity) was failed.")
        return False
    
    while True:
        time.sleep(0.001)
        status_result, axis_status = FAS_GetAxisStatus(nBdID)
        if status_result != FMM_OK:
            print("Function(FAS_GetAxisStatus) was failed.")
            return False

        if not (axis_status & EZISERVO2_AXISSTATUS.FFLAG_MOTIONING) and (
            axis_status & EZISERVO2_AXISSTATUS.FFLAG_INPOSITION
        ):
            break

    return True


# 멀티쓰레딩으로 주기적 송신 실행
send_thread = threading.Thread(target=send_can_message)
send_thread.daemon = True  # 메인 스레드 종료 시 함께 종료되도록 설정
send_thread.start()

# multi threading Receive Message Run
receive_thread = threading.Thread(target=receive_can_message)
receive_thread.daemon = True
receive_thread.start()


# 메인 스레드 유지 (컨트롤 + 씨 로 종료 가능)
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")
    # Connection Close
    for i in range(2):
        FAS_Close(i)

    input("Press Enter to exit...")
    os.system('sudo ip link set can0 down')

