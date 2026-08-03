import sys

from PySide6.QtWidgets import QApplication

from .analyzers.arm_runner import shutdown_rclpy
from .main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    # Names are required for QSettings to find the persisted INI file.
    app.setOrganizationName("openarm")
    app.setApplicationName("OpenArm Launcher")
    window = MainWindow()
    window.show()
    app.aboutToQuit.connect(shutdown_rclpy)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
