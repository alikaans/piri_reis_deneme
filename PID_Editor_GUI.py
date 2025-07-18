from PyQt5.QtWidgets import (QMainWindow, QApplication, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QFormLayout, QDoubleSpinBox, 
                            QPushButton, QStatusBar, QLabel)
from PyQt5.QtCore import QTimer
from serial_manager import SerialPortManager
import sys

class Window1(QMainWindow):
    def __init__(self, serial_manager):
        super().__init__()
        self.serial_manager = serial_manager
        self.setWindowTitle("PID Kontrol Arayüzü")
        self.initUI()
        
    def initUI(self):
        # Ana widget ve layout
        main_widget = QWidget()
        main_layout = QVBoxLayout()
        
        # PID Kontrolleri için grup kutuları
        self.yaw_group = self.create_pid_group("Yaw")
        self.pitch_group = self.create_pid_group("Pitch")
        self.roll_group = self.create_pid_group("Roll")

        # Eksenleri yan yana yerleştirme
        axes_layout = QHBoxLayout()
        axes_layout.addWidget(self.yaw_group)
        axes_layout.addWidget(self.pitch_group)
        axes_layout.addWidget(self.roll_group)

        # Gönder butonu
        self.btn_send_all = QPushButton("Tüm PID Değerlerini Gönder")
        self.btn_send_all.clicked.connect(self.send_all_pid_values)

        # Durum çubuğu ve bağlantı göstergesi
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.connection_led = QLabel("●")
        self.connection_led.setStyleSheet("font-size: 20px;")
        self.status_bar.addPermanentWidget(self.connection_led)

        # Layout'ları birleştirme
        main_layout.addLayout(axes_layout)
        main_layout.addWidget(self.btn_send_all)
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # Bağlantı kontrol timer'ı
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_connection_status)
        self.timer.start(1000)

    def create_pid_group(self, axis_name):
        """Her eksen için PID kontrol grubu oluşturur"""
        group = QGroupBox(f"{axis_name} PID")
        layout = QFormLayout()
        
        # SpinBox'ları oluştur
        spin_kp = QDoubleSpinBox()
        spin_ki = QDoubleSpinBox()
        spin_kd = QDoubleSpinBox()
        
        # Ortak ayarlar
        for spin in [spin_kp, spin_ki, spin_kd]:
            spin.setRange(0, 1000.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            spin.setValue(0.0)
        
        # Layout'a ekle
        layout.addRow(f"Kp:", spin_kp)
        layout.addRow(f"Ki:", spin_ki)
        layout.addRow(f"Kd:", spin_kd)
        
        group.setLayout(layout)
        
        # Nesnelere referans ekle
        setattr(self, f"sb_{axis_name.lower()}_kp", spin_kp)
        setattr(self, f"sb_{axis_name.lower()}_ki", spin_ki)
        setattr(self, f"sb_{axis_name.lower()}_kd", spin_kd)
        
        return group

    def update_connection_status(self):
        """Seri port bağlantı durumunu günceller"""
        if self.serial_manager.serial_port and self.serial_manager.serial_port.is_open:
            self.connection_led.setStyleSheet("color: green;")
        else:
            self.connection_led.setStyleSheet("color: red;")

    def get_pid_values(self):
        """Tüm PID değerlerini sözlük olarak döndürür"""
        return {
            'yaw': {
                'kp': self.sb_yaw_kp.value(),
                'ki': self.sb_yaw_ki.value(),
                'kd': self.sb_yaw_kd.value()
            },
            'pitch': {
                'kp': self.sb_pitch_kp.value(),
                'ki': self.sb_pitch_ki.value(),
                'kd': self.sb_pitch_kd.value()
            },
            'roll': {
                'kp': self.sb_roll_kp.value(),
                'ki': self.sb_roll_ki.value(),
                'kd': self.sb_roll_kd.value()
            }
        }

    def send_all_pid_values(self):
        """Tüm PID değerlerini seri porta gönderir"""
        pid_data = self.get_pid_values()
        command_str = "PID:"
        
        for axis in ['yaw', 'pitch', 'roll']:
            values = pid_data[axis]
            command_str += (
                f"{axis},"
                f"{values['kp']:.3f},"
                f"{values['ki']:.3f},"
                f"{values['kd']:.3f};"
            )
        
        command_str += "\n"
        self.serial_manager.send_data(command_str)
        print(f"Gönderilen komut: {command_str}")