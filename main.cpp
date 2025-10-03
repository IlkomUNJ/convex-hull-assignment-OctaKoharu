#include <QApplication>
#include "mainwindow.h"

// Fungsi utama aplikasi Qt.
int main(int argc, char *argv[])
{
    // Membuat objek aplikasi.
    QApplication a(argc, argv);

    // Membuat dan menampilkan jendela utama (MainWindow).
    MainWindow w;
    w.show();

    // Memulai loop eksekusi aplikasi.
    return a.exec();
}
