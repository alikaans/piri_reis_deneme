from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QTimer
from serial_manager import SerialPortManager

class OtonomStarter(QMainWindow):
    def __init__(self, serial_manager):
        super().__init__()
        self.serial_manager = serial_manager
        self.is_autonomous = False
        self.setWindowTitle("Otonom Mod Kontrolü")
        self.setFixedSize(300, 200)
        self.init_ui()

    def init_ui(self):
        # Ana widget ve layout
        main_widget = QWidget()
        main_layout = QVBoxLayout()

        # Otonom mod başlat/durdur düğmesi
        self.btn_autonomous = QPushButton("Otonom Mod Başlat")
        self.btn_autonomous.setStyleSheet("font-size: 14px; padding: 10px;")
        self.btn_autonomous.clicked.connect(self.toggle_autonomous_mode)

        # Rota yükleme düğmesi
        self.btn_load_route = QPushButton("Rota Yükle")
        self.btn_load_route.setStyleSheet("font-size: 14px; padding: 10px;")
        self.btn_load_route.clicked.connect(self.load_route)

        # Durum etiketi
        self.status_label = QLabel("Durum: Otonom mod kapalı")
        self.status_label.setStyleSheet("font-size: 12px;")

        # Layout'a ekle
        main_layout.addWidget(self.btn_autonomous)
        main_layout.addWidget(self.btn_load_route)
        main_layout.addWidget(self.status_label)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # Veri okuma için timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(1000)  # Her saniye veri oku

    def toggle_autonomous_mode(self):
        """Otonom modu başlatır veya durdurur"""
        if not self.is_autonomous:
            command = "AUTONOM:START\n"
            self.serial_manager.send_data(command)
            self.btn_autonomous.setText("Otonom Modu Durdur")
            self.status_label.setText("Durum: Otonom mod aktif")
            self.is_autonomous = True
        else:
            command = "AUTONOM:STOP\n"
            self.serial_manager.send_data(command)
            self.btn_autonomous.setText("Otonom Mod Başlat")
            self.status_label.setText("Durum: Otonom mod kapalı")
            self.is_autonomous = False

    def load_route(self):
        """Önceden tanımlı bir rotayı yükler"""
        command = "AUTONOM:LOAD_ROUTE:DEFAULT\n"  # Suibo'nun rota formatına göre güncellenebilir
        self.serial_manager.send_data(command)
        self.status_label.setText("Durum: Rota yüklendi")

    def update_status(self):
        """Seri porttan gelen verileri oku ve durumu güncelle"""
        data = self.serial_manager.read_data()
        if data:
            self.status_label.setText(f"Durum: {data}")