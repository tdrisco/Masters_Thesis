import MTI_Output as MTI
import livePlot as LP



def main():

    app = LP.QtWidgets.QApplication(LP.sys.argv)
    w = LP.MainWindow()
    w.show()
    LP.sys.exit(app.exec_())
    


if __name__ == '__main__':
    main()
