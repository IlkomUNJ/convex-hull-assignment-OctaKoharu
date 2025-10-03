#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QPoint>
#include <QList>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMouseEvent>

// Enum untuk memilih algoritma Convex Hull
enum class HullAlgorithm {
    Slow,
    Fast // Graham Scan
};

// Kelas DrawingWidget: Canvas untuk menggambar titik dan Convex Hull
class DrawingWidget : public QWidget {
    Q_OBJECT

public:
    explicit DrawingWidget(QWidget *parent = nullptr);
    void setAlgorithm(HullAlgorithm alg);

public slots:
    void runConvexHull(); // Slot untuk menjalankan Convex Hull
    void clearCanvas();   // Slot untuk menghapus kanvas

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;

private:
    QList<QPoint> points;           // Daftar titik yang digambar pengguna
    QList<QPoint> hullPoints;       // Daftar titik yang membentuk Convex Hull
    int iterationCount = 0;         // Penghitung iterasi algoritma
    HullAlgorithm currentAlgorithm = HullAlgorithm::Slow; // Algoritma yang akan dijalankan

    // --- Helper Functions untuk Geometri Komputasional ---

    // Fungsi untuk menghitung orientasi (cross product 2D) dari tripel (p, q, r).
    // Digunakan untuk menentukan belokan (kiri/kanan/kolinear).
    // > 0: Belok ke kiri (Counter-clockwise)
    // = 0: Kolinear
    // < 0: Belok ke kanan (Clockwise)
    int orientation(const QPoint &p, const QPoint &q, const QPoint &r);

    // Fungsi untuk menghitung Convex Hull dengan metode Slow (Brute Force)
    QList<QPoint> convexHullSlow(const QList<QPoint> &inputPoints);

    // Fungsi untuk menghitung Convex Hull dengan metode Fast (Graham Scan)
    QList<QPoint> convexHullFast(const QList<QPoint> &inputPoints);
};

// Kelas MainWindow: Jendela utama yang menampung canvas dan tombol
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void runSlowHull();
    void runFastHull();

private:
    DrawingWidget *drawingWidget;
};

#endif // MAINWINDOW_H
