#include "mainwindow.h"
#include <QMessageBox>
#include <algorithm>
#include <cmath>

// ==========================================================
// IMPLEMENTASI DRAWINGWIDGET
// ==========================================================

DrawingWidget::DrawingWidget(QWidget *parent)
    : QWidget(parent)
{
    // Mengatur palet untuk warna latar belakang putih
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);
}

void DrawingWidget::setAlgorithm(HullAlgorithm alg)
{
    currentAlgorithm = alg;
}

// Menghitung orientasi tripel (p, q, r)
// > 0: CCW, < 0: CW, = 0: Collinear
int DrawingWidget::orientation(const QPoint &p, const QPoint &q, const QPoint &r)
{
    // Rumus cross product 2D: (qy - py) * (rx - qx) - (qx - px) * (ry - qy)
    qreal val = (q.y() - p.y()) * (r.x() - q.x()) -
                (q.x() - p.x()) * (r.y() - q.y());

    if (qAbs(val) < 1e-9) return 0;  // Kolinear
    return (val > 0) ? 1 : 2;       // 1: CCW (Kiri), 2: CW (Kanan)
}

// Algoritma Slow Convex Hull (Brute Force O(n^3))
QList<QPoint> DrawingWidget::convexHullSlow(const QList<QPoint> &inputPoints)
{
    QList<QPoint> hull;
    iterationCount = 0;

    if (inputPoints.size() < 3) return inputPoints;

    const int n = inputPoints.size();

    // Periksa setiap pasangan titik (pi, pj) sebagai calon segmen hull.
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;

            bool isHullEdge = true;
            bool hasPointOnLeft = false;
            bool hasPointOnRight = false;

            // Periksa apakah semua titik lainnya (pk) berada di sisi yang sama dari segmen (pi, pj).
            for (int k = 0; k < n; ++k) {
                iterationCount++; // Hitung iterasi perbandingan tripel

                if (k == i || k == j) continue;

                int o = orientation(inputPoints[i], inputPoints[j], inputPoints[k]);

                if (o == 1) { // CCW (Kiri)
                    hasPointOnLeft = true;
                } else if (o == 2) { // CW (Kanan)
                    hasPointOnRight = true;
                }
            }

            // Jika semua titik k lainnya berada di satu sisi (kanan atau kiri),
            // maka segmen (pi, pj) adalah bagian dari Convex Hull.
            // Kita hanya perlu memastikan semua titik berada di sisi KANAN/CW (atau kolinear)
            // dari segmen (pi, pj) yang diurutkan (CW order untuk CH).
            // Atau, kita pastikan TIDAK ADA titik di kedua sisi secara bersamaan.
            if (!hasPointOnLeft || !hasPointOnRight) {
                // Untuk memastikan urutan CCW, kita hanya menyimpan jika semua sisanya
                // berada di sisi kanan/CW (atau kolinear).
                // Karena kita belum mengurutkan, kita hanya menambahkan kedua titik
                // dan akan mengurutkannya nanti, atau hanya memastikan segmen (pi, pj) valid.

                // Pendekatan yang lebih sederhana:
                // Jika semua titik lain berada di sisi KANAN/CW (atau kolinear) dari (pi, pj),
                // maka (pi, pj) adalah segmen hull (upper/lower-hull tergantung arah).

                // Mari kita gunakan definisi "semua titik lain di satu sisi" untuk mengidentifikasi
                // segmen. Kemudian urutkan titik-titik hull yang ditemukan.

                if (hasPointOnLeft ^ hasPointOnRight) { // Hanya ada di satu sisi (tidak termasuk kolinear)
                    if (!hull.contains(inputPoints[i])) hull.append(inputPoints[i]);
                    if (!hull.contains(inputPoints[j])) hull.append(inputPoints[j]);
                } else if (!hasPointOnLeft && !hasPointOnRight) {
                    // Semua titik kolinear dengan segmen, ini kasus degenerasi, tambahkan saja.
                    if (!hull.contains(inputPoints[i])) hull.append(inputPoints[i]);
                    if (!hull.contains(inputPoints[j])) hull.append(inputPoints[j]);
                }
            }
        }
    }

    // Titik-titik hull harus diurutkan (misalnya berdasarkan sudut)
    if (hull.size() > 2) {
        // Temukan titik P0 (kiri-bawah)
        QPoint p0 = *std::min_element(hull.begin(), hull.end(),
                                      [](const QPoint &a, const QPoint &b) {
                                          if (a.y() != b.y()) return a.y() < b.y();
                                          return a.x() < b.x();
                                      });

        // Urutkan berdasarkan sudut relatif terhadap P0
        std::sort(hull.begin(), hull.end(),
                  [p0](const QPoint &a, const QPoint &b) {
                      if (a == p0) return true;
                      if (b == p0) return false;

                      qreal angleA = std::atan2(a.y() - p0.y(), a.x() - p0.x());
                      qreal angleB = std::atan2(b.y() - p0.y(), b.x() - p0.x());
                      return angleA < angleB;
                  });
    }

    // Hapus duplikat yang mungkin terjadi dari proses Brute Force
    QList<QPoint> uniqueHull;
    for (const QPoint& p : hull) {
        if (uniqueHull.isEmpty() || uniqueHull.last() != p) {
            uniqueHull.append(p);
        }
    }
    if (uniqueHull.size() > 1 && uniqueHull.first() == uniqueHull.last()) {
        uniqueHull.removeLast();
    }

    return uniqueHull;
}

// Algoritma Fast Convex Hull (Graham Scan O(n log n))
QList<QPoint> DrawingWidget::convexHullFast(const QList<QPoint> &inputPoints)
{
    QList<QPoint> hull;
    iterationCount = 0;

    if (inputPoints.size() < 3) return inputPoints;

    // 1. Temukan titik awal P0 (kiri-bawah)
    QPoint p0 = inputPoints.first();
    for (const QPoint &p : inputPoints) {
        if (p.y() < p0.y() || (p.y() == p0.y() && p.x() < p0.x())) {
            p0 = p;
        }
    }

    // Salin dan hapus P0 dari daftar
    QList<QPoint> sortedPoints = inputPoints;
    sortedPoints.removeAll(p0);

    // 2. Urutkan titik berdasarkan sudut relatif terhadap P0
    // Menggunakan fungsi perbandingan untuk std::sort
    std::sort(sortedPoints.begin(), sortedPoints.end(),
              [this, p0](const QPoint &a, const QPoint &b) {
                  iterationCount++; // Hitung iterasi perbandingan

                  int o = orientation(p0, a, b);
                  if (o == 0) {
                      // Jika kolinear, yang terdekat dengan p0 didahulukan
                      return QPoint::dotProduct(a - p0, a - p0) < QPoint::dotProduct(b - p0, b - p0);
                  }
                  return (o == 1); // Belok kiri (CCW) didahulukan
              });

    // 3. Masukkan P0 dan dua titik berikutnya ke hull (stack)
    hull.append(p0);
    hull.append(sortedPoints[0]);

    // Pastikan setidaknya ada 3 titik yang akan diproses
    if (sortedPoints.size() < 2) return inputPoints;

    hull.append(sortedPoints[1]);

    // 4. Lakukan pemindaian (scan)
    for (int i = 2; i < sortedPoints.size(); ++i) {
        iterationCount++; // Hitung iterasi pemindaian utama

        // Hapus titik terakhir dari hull selama triple terakhir (sebelum titik terakhir, titik terakhir, titik berikutnya)
        // tidak membentuk belokan kiri (CCW).
        while (hull.size() > 1 && orientation(hull[hull.size() - 2], hull.last(), sortedPoints[i]) != 1) {
            hull.removeLast();
            iterationCount++; // Hitung iterasi pop
        }
        hull.append(sortedPoints[i]);
    }

    return hull;
}


// SLOT: Menjalankan Convex Hull dan memperbarui tampilan
void DrawingWidget::runConvexHull()
{
    hullPoints.clear();
    iterationCount = 0;

    if (points.size() < 3) {
        QMessageBox::information(this, "Informasi", "Silakan gambar minimal 3 titik di kanvas.");
        update();
        return;
    }

    if (currentAlgorithm == HullAlgorithm::Slow) {
        hullPoints = convexHullSlow(points);
        qDebug() << "Slow Hull Iterations:" << iterationCount;
    } else { // HullAlgorithm::Fast
        hullPoints = convexHullFast(points);
        qDebug() << "Fast Hull Iterations (Graham Scan):" << iterationCount;
    }

    update(); // Minta kanvas untuk menggambar ulang
}

// SLOT: Menghapus semua titik dan hasil hull
void DrawingWidget::clearCanvas()
{
    points.clear();
    hullPoints.clear();
    iterationCount = 0;
    update();
}

// Menangani klik mouse untuk menambahkan titik
void DrawingWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        points.append(event->pos());
        hullPoints.clear(); // Hapus hasil sebelumnya saat menambahkan titik baru
        iterationCount = 0;
        update();
    }
}

// Menggambar titik, hull, dan teks status
void DrawingWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 1. Gambar Titik-titik
    painter.setPen(QPen(Qt::black, 8, Qt::SolidLine, Qt::RoundCap));
    for (const QPoint &p : points) {
        painter.drawPoint(p);
    }

    // 2. Gambar Convex Hull
    if (hullPoints.size() >= 2) {
        painter.setPen(QPen(Qt::red, 3, Qt::SolidLine, Qt::RoundCap));

        // Garis antara titik-titik hull
        for (int i = 0; i < hullPoints.size(); ++i) {
            painter.drawLine(hullPoints[i], hullPoints[(i + 1) % hullPoints.size()]);
        }

        // Tandai titik-titik hull
        painter.setPen(QPen(Qt::blue, 10, Qt::SolidLine, Qt::RoundCap));
        for (const QPoint &p : hullPoints) {
            painter.drawPoint(p);
        }
    }

    // 3. Gambar Status (Iterasi dan Algoritma)
    painter.setPen(QPen(Qt::darkCyan, 1));
    painter.setFont(QFont("Arial", 12));

    QString algoName = (currentAlgorithm == HullAlgorithm::Slow) ?
                           "Slow Convex Hull (O(n^3) - Brute Force)" :
                           "Fast Convex Hull (O(n log n) - Graham Scan)";

    // Teks di sudut kiri atas
    painter.drawText(10, 20, "Titik: " + QString::number(points.size()));
    painter.drawText(10, 40, "Algoritma: " + algoName);

    if (iterationCount > 0) {
        painter.drawText(10, 60, "Iterasi: " + QString::number(iterationCount));
    } else if (points.size() > 0) {
        painter.drawText(10, 60, "Klik 'Jalankan' untuk simulasi.");
    } else {
        painter.drawText(10, 60, "Klik di kanvas untuk menambahkan titik.");
    }
}


// ==========================================================
// IMPLEMENTASI MAINWINDOW
// ==========================================================

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("Aplikasi Convex Hull (Qt Widgets)");

    // 1. Central Widget & Main Layout
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    // 2. Drawing Widget (Canvas)
    drawingWidget = new DrawingWidget(this);
    // Beri ukuran minimum agar kanvas terlihat jelas
    drawingWidget->setMinimumSize(800, 600);
    mainLayout->addWidget(drawingWidget);

    // 3. Layout untuk Tombol
    QHBoxLayout *buttonLayout = new QHBoxLayout();

    // 4. Tombol-tombol

    // Tombol untuk Slow Convex Hull
    QPushButton *runSlowButton = new QPushButton("Jalankan Slow Hull", this);
    connect(runSlowButton, &QPushButton::clicked, this, &MainWindow::runSlowHull);

    // Tombol untuk Fast Convex Hull
    QPushButton *runFastButton = new QPushButton("Jalankan Fast Hull", this);
    connect(runFastButton, &QPushButton::clicked, this, &MainWindow::runFastHull);

    // Tombol Clear Canvas
    QPushButton *clearButton = new QPushButton("Bersihkan Kanvas", this);
    connect(clearButton, &QPushButton::clicked, drawingWidget, &DrawingWidget::clearCanvas);

    // Tambahkan tombol ke layout horizontal
    buttonLayout->addWidget(runSlowButton);
    buttonLayout->addWidget(runFastButton);
    buttonLayout->addWidget(clearButton);

    // Tambahkan layout tombol ke layout utama
    mainLayout->addLayout(buttonLayout);
}

MainWindow::~MainWindow()
{
}

// SLOT: Mengatur algoritma ke Slow dan menjalankan
void MainWindow::runSlowHull()
{
    drawingWidget->setAlgorithm(HullAlgorithm::Slow);
    drawingWidget->runConvexHull();
}

// SLOT: Mengatur algoritma ke Fast dan menjalankan
void MainWindow::runFastHull()
{
    drawingWidget->setAlgorithm(HullAlgorithm::Fast);
    drawingWidget->runConvexHull();
}
