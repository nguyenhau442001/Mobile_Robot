#!/usr/bin/env python3
"""GUI editor for robot dimensions configuration YAML file."""

import atexit
from pathlib import Path
import re
import signal
import sys

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QLocale
from PyQt5.QtCore import QPoint
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import QPolygon
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QDoubleSpinBox
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QScrollArea
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from ruamel.yaml import YAML

CONFIG_PATH = (
    Path(get_package_share_directory('mobile_robot_description'))
    / 'config'
    / 'yuhou'
    / 'robot_dimensions_config.yaml'
)

_VALUE_RE = re.compile(r'^(\s+\S+\s*:\s*)([+-]?\d+(?:\.\d*)?)(\s*(?:#.*)?)$')
_SECTION_RE = re.compile(r'^(\S[^:]*):')
_SCRIPT_DIR = Path(__file__).resolve().parent


def _make_arrow_png(up: bool) -> str:
    """Render a white triangle arrow PNG into images/; delete it on exit."""
    images_dir = _SCRIPT_DIR / 'images'
    images_dir.mkdir(exist_ok=True)
    path = images_dir / ('arrow_up.png' if up else 'arrow_dn.png')
    px = QPixmap(10, 10)
    px.fill(Qt.transparent)
    p = QPainter(px)
    p.setRenderHint(QPainter.Antialiasing)
    p.setBrush(QColor('#ffffff'))
    p.setPen(Qt.NoPen)
    if up:
        p.drawPolygon(QPolygon([QPoint(1, 8), QPoint(9, 8), QPoint(5, 2)]))
    else:
        p.drawPolygon(QPolygon([QPoint(1, 2), QPoint(9, 2), QPoint(5, 8)]))
    p.end()
    px.save(str(path), 'PNG')
    atexit.register(path.unlink, missing_ok=True)
    return str(path)


class _SpinBox(QDoubleSpinBox):
    """QDoubleSpinBox that ignores mouse wheel to prevent accidental value changes."""

    def wheelEvent(self, event):
        event.ignore()


APP_STYLE = """
QWidget {
    background-color: #f5f5f5;
    color: #212121;
    font-family: 'Segoe UI', Arial, sans-serif;
    font-size: 13px;
}
QScrollArea {
    border: none;
    background-color: #f5f5f5;
}
QScrollBar:vertical {
    background-color: #e0e0e0;
    width: 12px;
    border-radius: 6px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background-color: #1976d2;
    border-radius: 6px;
    min-height: 30px;
}
QScrollBar::handle:vertical:hover {
    background-color: #1e88e5;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
    background: none;
}
QGroupBox {
    border: 1px solid #c8c8c8;
    border-radius: 8px;
    margin-top: 20px;
    padding: 16px 8px 8px 8px;
    font-weight: bold;
    font-size: 13px;
    color: #1565c0;
    background-color: #ffffff;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 8px;
    left: 14px;
    top: 2px;
    background-color: #ffffff;
    border-radius: 4px;
}
QLabel {
    color: #424242;
    min-width: 180px;
    background-color: transparent;
}
QDoubleSpinBox {
    background-color: #ffffff;
    border: 1px solid #bdbdbd;
    border-radius: 5px;
    padding: 4px 8px;
    color: #212121;
    min-width: 110px;
    max-width: 140px;
}
QDoubleSpinBox:focus {
    border: 1px solid #1976d2;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    width: 22px;
    background-color: #1976d2;
    border-radius: 3px;
}
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
    background-color: #1e88e5;
}
QDoubleSpinBox::up-button:pressed, QDoubleSpinBox::down-button:pressed {
    background-color: #1565c0;
}
QDoubleSpinBox::up-arrow {
    image: url(UP_ARROW_PATH);
    width: 10px;
    height: 10px;
}
QDoubleSpinBox::down-arrow {
    image: url(DN_ARROW_PATH);
    width: 10px;
    height: 10px;
}
"""

BUTTON_BAR_STYLE = """
QWidget#buttonBar {
    background-color: #eeeeee;
    border-top: 1px solid #c8c8c8;
}
"""

RELOAD_BTN_STYLE = """
QPushButton {
    background-color: #ffffff;
    color: #424242;
    border: 1px solid #bdbdbd;
    border-radius: 7px;
    padding: 8px 24px;
    font-size: 13px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #e3f2fd;
    border-color: #1976d2;
    color: #1565c0;
}
QPushButton:pressed {
    background-color: #bbdefb;
}
"""

SAVE_BTN_STYLE = """
QPushButton {
    background-color: #1976d2;
    color: #ffffff;
    border: none;
    border-radius: 7px;
    padding: 8px 28px;
    font-size: 13px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #1e88e5;
}
QPushButton:pressed {
    background-color: #1565c0;
}
"""


def _patch_yaml_line(line, new_value):
    """Return line with its numeric value replaced; comment and spacing unchanged."""
    m = _VALUE_RE.match(line)
    if not m:
        return line
    suffix = '\n' if line.endswith('\n') else ''
    return m.group(1) + f'{new_value:.10g}' + m.group(3) + suffix


class RobotConfigEditor(QWidget):
    """Widget for viewing and editing robot dimension configuration values."""

    def __init__(self):
        """Initialize the editor window and load configuration."""
        super().__init__()

        self.setWindowTitle('Robot Dimensions Config Editor')
        self.setStyleSheet(APP_STYLE)
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(720, screen.height())

        self._yaml = YAML()
        self._yaml.preserve_quotes = True
        self.config_data = {}
        self.spinboxes = {}
        self.original_values = {}

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        self.content_widget = QWidget()
        self.content_layout = QVBoxLayout(self.content_widget)
        self.content_layout.setContentsMargins(16, 16, 16, 16)
        self.content_layout.setSpacing(12)

        self.scroll_area.setWidget(self.content_widget)
        self.main_layout.addWidget(self.scroll_area)

        button_bar = QWidget()
        button_bar.setObjectName('buttonBar')
        button_bar.setStyleSheet(BUTTON_BAR_STYLE)
        button_layout = QHBoxLayout(button_bar)
        button_layout.setContentsMargins(20, 12, 20, 12)
        button_layout.setSpacing(10)

        self.reload_btn = QPushButton('↺   Reload')
        self.reload_btn.setStyleSheet(RELOAD_BTN_STYLE)
        self.reload_btn.setMinimumHeight(38)
        self.reload_btn.setCursor(Qt.PointingHandCursor)

        self.save_btn = QPushButton('Save')
        self.save_btn.setStyleSheet(SAVE_BTN_STYLE)
        self.save_btn.setMinimumHeight(38)
        self.save_btn.setCursor(Qt.PointingHandCursor)

        self.reload_btn.clicked.connect(self.load_config)
        self.save_btn.clicked.connect(self.save_config)

        button_layout.addStretch()
        button_layout.addWidget(self.reload_btn)
        button_layout.addWidget(self.save_btn)

        self.main_layout.addWidget(button_bar)

        self.load_config()

    def _validate_mass(self, spinbox, param_name):
        """Warn and reset if a mass parameter is set to zero or below."""
        if spinbox.value() <= 0:
            QMessageBox.warning(
                self,
                'Invalid Value',
                f'"{param_name}" must be greater than 0.\nValue has been reset to 0.0001.',
            )
            spinbox.setValue(0.0001)

    def create_spinbox(self, value):
        """Create and return a configured QDoubleSpinBox for a parameter value."""
        spinbox = _SpinBox()
        spinbox.setDecimals(4)
        spinbox.setRange(0.0, 1000.0)
        spinbox.setSingleStep(0.001)
        spinbox.setLocale(QLocale(QLocale.C))
        spinbox.setValue(float(value))
        return spinbox

    def clear_form(self):
        """Remove all widgets from the content layout and clear spinbox references."""
        while self.content_layout.count():
            item = self.content_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        self.spinboxes.clear()
        self.original_values.clear()

    def load_config(self):
        """Load configuration from YAML file and populate the form."""
        try:
            with open(CONFIG_PATH, 'r') as f:
                self.config_data = self._yaml.load(f)

            self.clear_form()

            for section_name, section_values in self.config_data.items():

                group_box = QGroupBox(section_name)
                group_layout = QVBoxLayout()
                group_layout.setSpacing(8)

                self.spinboxes[section_name] = {}
                self.original_values[section_name] = {}

                for param_name, param_value in section_values.items():

                    row = QHBoxLayout()
                    row.setContentsMargins(4, 2, 4, 2)

                    label = QLabel(param_name)
                    spinbox = self.create_spinbox(param_value)

                    if param_name.endswith('_mass'):
                        spinbox.setMinimum(0.0001)
                        spinbox.editingFinished.connect(
                            lambda sb=spinbox, pn=param_name: self._validate_mass(sb, pn)
                        )

                    row.addWidget(label)
                    row.addStretch()
                    row.addWidget(spinbox)

                    group_layout.addLayout(row)

                    self.spinboxes[section_name][param_name] = spinbox
                    self.original_values[section_name][param_name] = float(param_value)

                group_box.setLayout(group_layout)
                self.content_layout.addWidget(group_box)

            self.content_layout.addStretch()

        except Exception as e:
            QMessageBox.critical(self, 'Load Error', str(e))

    def save_config(self):
        """Patch only changed lines in the YAML file, preserving all formatting."""
        try:
            changes = {}
            for section_name, params in self.spinboxes.items():
                for param_name, spinbox in params.items():
                    new_value = spinbox.value()
                    if new_value != self.original_values[section_name][param_name]:
                        changes[(section_name, param_name)] = new_value

            if not changes:
                QMessageBox.information(self, 'No Changes', 'No values were modified.')
                return

            lines = CONFIG_PATH.read_text().splitlines(keepends=True)
            current_section = None
            remaining = set(changes)
            for i, line in enumerate(lines):
                sec_m = _SECTION_RE.match(line)
                if sec_m:
                    current_section = sec_m.group(1).strip()
                    continue
                if not _VALUE_RE.match(line):
                    continue
                key = line.lstrip().split(':')[0].strip()
                lookup = (current_section, key)
                if lookup in remaining:
                    lines[i] = _patch_yaml_line(line, changes[lookup])
                    remaining.discard(lookup)
                    if not remaining:
                        break

            CONFIG_PATH.write_text(''.join(lines))

            for section_name, params in self.spinboxes.items():
                for param_name, spinbox in params.items():
                    self.original_values[section_name][param_name] = spinbox.value()

            QMessageBox.information(self, 'Saved', 'Configuration saved successfully.')

        except Exception as e:
            QMessageBox.critical(self, 'Save Error', str(e))


def main():
    """Entry point: launch the Qt application and open the config editor."""
    global APP_STYLE
    app = QApplication(sys.argv)

    APP_STYLE = (
        APP_STYLE
        .replace('UP_ARROW_PATH', _make_arrow_png(True))
        .replace('DN_ARROW_PATH', _make_arrow_png(False))
    )

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    timer = QTimer()
    timer.start(200)
    timer.timeout.connect(lambda: None)

    editor = RobotConfigEditor()
    editor.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
