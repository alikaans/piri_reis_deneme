import sys
from PyQt5.QtWidgets import QApplication
from PID_Editor_GUI import Window1
from Controller_Input_GUI import Window2
from otonom_starter import OtonomStarter
from serial_manager import SerialPortManager

def main():
    app = QApplication(sys.argv)
    
    # Seri port yöneticisini oluştur (COM portu ve baud rate'i ayarla)
    serial_manager = SerialPortManager(port='COM1', baudrate=115200)
    
    # Arayüz pencerelerini oluştur ve seri port yöneticisini ilet
    window1 = Window1(serial_manager)
    window2 = Window2(serial_manager)
    window3 = OtonomStarter(serial_manager)
    
    window1.show()
    window2.show()
    window3.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()