import serial
from threading import Lock
import time

class SerialPortManager:
    def __init__(self, port, baudrate=115200):
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.lock = Lock()
        self.open_connection()

    def open_connection(self):
        """Seri port bağlantısını aç"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=2
            )
            print(f"Bağlantı açıldı: {self.port}, {self.baudrate}")
        except serial.SerialException as e:
            print(f"Port açılamadı: {e}")
            self.serial_port = None

    def reconnect(self):
        """Bağlantıyı yeniden deneme"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        for _ in range(3):  # 3 kez yeniden bağlanmayı dene
            try:
                self.serial_port = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=2
                )
                print(f"Yeniden bağlanıldı: {self.port}")
                return True
            except serial.SerialException as e:
                print(f"Yeniden bağlantı hatası: {e}")
                time.sleep(1)
        return False

    def send_data(self, data):
        """Veriyi thread-safe şekilde gönder"""
        if not self.serial_port or not self.serial_port.is_open:
            print("Port bağlı değil, yeniden bağlanılıyor...")
            if not self.reconnect():
                print("Yeniden bağlantı başarısız!")
                return
        with self.lock:
            try:
                self.serial_port.write(data.encode('utf-8'))
                print(f"Gönderilen veri: {data}")
            except Exception as e:
                print(f"Gönderim hatası: {e}")
                self.reconnect()

    def read_data(self):
        """Seri porttan veri oku"""
        if not self.serial_port or not self.serial_port.is_open:
            print("Port bağlı değil, yeniden bağlanılıyor...")
            if not self.reconnect():
                return None
        with self.lock:
            try:
                data = self.serial_port.readline().decode('utf-8').strip()
                if data:
                    print(f"Alınan veri: {data}")
                return data
            except Exception as e:
                print(f"Okuma hatası: {e}")
                self.reconnect()
                return None

    def close_connection(self):
        """Bağlantıyı kapat"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Bağlantı kapatıldı")