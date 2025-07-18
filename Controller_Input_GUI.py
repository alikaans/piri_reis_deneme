from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QGroupBox, QLabel, QPushButton, QStatusBar, QProgressBar)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
import pygame
from serial_manager import SerialPortManager
import sys
import time

class GamepadThread(QThread):
    data_updated = pyqtSignal(object)  # Gamepad verilerini taşıyan sinyal

    def __init__(self):
        super().__init__()
        self.running = True
        self.deadzone = 0.1  # Joyistik ölü bölge

    def run(self):
        pygame.init()
        pygame.joystick.init()
        
        while self.running:
            if pygame.joystick.get_count() > 0:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                
                pygame.event.pump()
                axes = [round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())]
                buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
                
                # Ölü bölge uygula
                axes = [0.0 if abs(a) < self.deadzone else a for a in axes]
                
                self.data_updated.emit({
                    'axes': axes,
                    'buttons': buttons,
                    'connected': True
                })
            else:
                self.data_updated.emit({'connected': False})
            
            time.sleep(0.02)  # 50 Hz okuma hızı

        pygame.quit()

    def stop(self):
        self.running = False

class Window2(QMainWindow):
    def __init__(self, serial_manager):
        super().__init__()
        self.serial_manager = serial_manager
        self.setWindowTitle("Gamepad Kontrol Arayüzü")
        self.initUI()
        self.initGamepad()

    def initUI(self):
        # Ana widget ve layout
        main_widget = QWidget()
        main_layout = QVBoxLayout()

        # Gamepad veri göstergeleri
        self.axis_group = self.create_axis_group()
        self.button_group = self.create_button_group()
        
        # Kontroller
        control_layout = QHBoxLayout()
        self.btn_calibrate = QPushButton("Kalibrasyon Yap")
        self.btn_calibrate.clicked.connect(self.calibrate_gamepad)
        control_layout.addWidget(self.btn_calibrate)

        # Durum çubuğu
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.connection_label = QLabel("Gamepad: Bağlı Değil")
        self.status_bar.addPermanentWidget(self.connection_label)

        # Layout'ları birleştir
        main_layout.addWidget(self.axis_group)
        main_layout.addWidget(self.button_group)
        main_layout.addLayout(control_layout)
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # Seri port kontrol timer'ı
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_gamepad_data)
        self.timer.start(20)  # 50 Hz gönderim

        self.gamepad_data = {'axes': [], 'buttons': []}

    def create_axis_group(self):
        group = QGroupBox("Joyistikler ve Tetikler")
        layout = QHBoxLayout()
        
        self.axis_bars = []
        for i in range(4):  # İlk 4 eksen (Sol X/Y, Sağ X/Y)
            bar = QProgressBar()
            bar.setFormat(f"Eksen {i+1}")
            bar.setRange(-100, 100)
            bar.setValue(0)
            bar.setOrientation(Qt.Vertical)
            layout.addWidget(bar)
            self.axis_bars.append(bar)
            
        group.setLayout(layout)
        return group

    def create_button_group(self):
        group = QGroupBox("Düğmeler")
        self.button_layout = QHBoxLayout()
        
        self.button_labels = []
        for i in range(12):  # İlk 12 düğme
            label = QLabel(f"B{i+1}")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("background: gray; border-radius: 5px;")
            self.button_layout.addWidget(label)
            self.button_labels.append(label)
            
        group.setLayout(self.button_layout)
        return group

    def initGamepad(self):
        self.gamepad_thread = GamepadThread()
        self.gamepad_thread.data_updated.connect(self.update_gamepad_status)
        self.gamepad_thread.start()

    def update_gamepad_status(self, data):
        """Gamepad verilerini arayüze yansıt"""
        if data['connected']:
            self.connection_label.setText("Gamepad: Bağlı")
            self.connection_label.setStyleSheet("color: green;")
            
            # Eksenleri güncelle
            for i, value in enumerate(data['axes'][:4]):
                scaled_value = int(value * 100)
                self.axis_bars[i].setValue(scaled_value)
                
            # Düğmeleri güncelle
            for i, state in enumerate(data['buttons'][:12]):
                self.button_labels[i].setStyleSheet(
                    f"background: {'green' if state else 'gray'};"
                    "border-radius: 5px;"
                )
            
            self.gamepad_data = data
        else:
            self.connection_label.setText("Gamepad: Bağlı Değil")
            self.connection_label.setStyleSheet("color: red;")
            self.gamepad_data = {'axes': [], 'buttons': []}

    def calibrate_gamepad(self):
        """Kalibrasyon işlemi"""
        print("Kalibrasyon yapılıyor...")

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """Değer aralığını dönüştür"""
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def send_gamepad_data(self):
        """Gamepad verilerini seri porta gönder"""
        if not self.gamepad_data.get('connected', False):
            return

        try:
            # Veri formatı Suibo Derin Commander ile uyumlu olacak şekilde güncellendi
            axes = [self.map_value(a, -1.0, 1.0, 0, 255) for a in self.gamepad_data['axes']]
            buttons = self.gamepad_data['buttons']
            
            # Suibo için örnek format: !GP:AXES:[a1,a2,a3,a4];BUTTONS:[b1,b2,...];\n
            command = (
                f"!GP:"
                f"AXES:{axes[0]:03d},{axes[1]:03d},{axes[2]:03d},{axes[3]:03d};"
                f"BUTTONS:{','.join(str(b) for b in buttons[:8])};"
                "\n"
            )
            
            self.serial_manager.send_data(command)
            
        except Exception as e:
            print(f"Gönderim hatası: {e}")