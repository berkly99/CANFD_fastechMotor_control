# CANFD_fastechMotor_control
CANFD BRS 통신을 통해서 파스텍 plus-e 모델 제품의 제어를 위한 파이썬 코드

# RS232 RS485 CANFD waveshare board installation manual

## waveshare board wiki link
https://www.waveshare.com/wiki/RS232-RS485-CAN-Board

## PCIE TO M.2 Board (C)
https://www.waveshare.com/wiki/PCIe_TO_M.2_Board_(C)

### ssd 128GB 라즈비안 64bit 설치 메뉴얼 대로 진행

### 파스텍 리눅스 plus-E 라이브러리 다운로드 링크
https://fastech-motions.com/software/250107_Program_Plus-E%20Linux%20Library_Ver.1.0.3_KR.zip

#### python -> amd64 -> 250107_Program_Plus-E Linux_Library_Python_Ver.1.0.3_64bit.tar.gz
#### 파일 압축 해제 후 
#### python -> PE -> Library 폴더를 VScode 워크스페이스 폴더로 이동

#### sudo cp -df ./libEziMOTIONPlusE.so* /usr/local/lib
#### sudo ldconfig





## samba 설치 
## sudo nmtui 를 이용해서 eth0 포트 192.168.0.83 고정 ip로 변경