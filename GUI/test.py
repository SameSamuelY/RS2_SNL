import sys
from PyQt5.QtWidgets import QApplication, QLabel

app = QApplication(sys.argv)
label = QLabel("Hello")
label.show()
sys.exit(app.exec_())